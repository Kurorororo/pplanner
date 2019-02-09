#include "cuda_search/cuda_hetro_gbfs.cuh"

#include "evaluator.h"
#include "open_list_factory.h"
#include "cuda_common/cuda_check.cuh"
#include "cuda_search/cuda_random_walk.cuh"

namespace pplanner {

using std::vector;

extern __constant__ CudaSASPlus cuda_problem;
extern __constant__ CudaSuccessorGenerator cuda_generator;
extern __constant__ CudaLandmarkGraph cuda_landmark_graph;
extern __constant__ CudaZobristHash cuda_c_hash;
extern __constant__ CudaZobristHash cuda_d_hash;

std::size_t CudaHetroGBFS::AllocateMessage() {
  m_.h_min = new int[n_threads_];
  m_.states = new int[n_threads_ * problem_->n_variables()];
  m_.accepted = new uint8_t[n_threads_ * graph_->n_landmarks_bytes()];

  CUDA_CHECK(cudaMalloc((void**)&cuda_steps_, n_threads_ * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&cuda_offsets_, n_threads_ * sizeof(int)));

  return 2 * n_threads_ * sizeof(int);
}

void CudaHetroGBFS::Init(const boost::property_tree::ptree &pt) {
  if (auto opt = pt.get_optional<int>("n_grid"))
    n_grid_ = opt.get();

  if (auto opt = pt.get_optional<int>("n_block"))
    n_block_ = opt.get();

  n_threads_ = n_grid_ * n_block_;

  if (auto opt = pt.get_optional<int>("gpu_threshold"))
    gpu_threshold_ = opt.get();

  graph_->InitLandmarks(lmcount_->landmark_graph());

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram"))
    ram = opt.get();

  graph_->ReserveByRAMSize(ram);

  auto open_list_option = pt.get_child("open_list");
  open_ = OpenListFactory(open_list_option);

  std::size_t gpu_ram = 8000000000;

  if (auto opt = pt.get_optional<size_t>("gpu_ram"))
    gpu_ram = opt.get();

  gpu_ram -= InitCudaSASPlus(problem_, cuda_problem_);
  gpu_ram -= InitCudaSuccessorGenerator(generator_, cuda_generator_);
  gpu_ram -= InitCudaLandmarkGraph(lmcount_->landmark_graph(),
                                   cuda_landmark_graph_);
  gpu_ram -= InitCudaZobristHash(graph_->hash(), cuda_c_hash_);
  gpu_ram -= InitCudaZobristHash(d_hash_, cuda_d_hash_);

  CUDA_CHECK(cudaMemcpyToSymbol(cuda_problem, cuda_problem_,
                                sizeof(CudaSASPlus)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_generator, cuda_generator_,
                                sizeof(CudaSuccessorGenerator)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_landmark_graph, cuda_landmark_graph_,
                                sizeof(CudaLandmarkGraph)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_c_hash, cuda_c_hash_,
                                sizeof(CudaZobristHash)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_d_hash, cuda_d_hash_,
                                sizeof(CudaZobristHash)));

  gpu_ram -= CudaInitializeOpenList(n_threads_,
                                    lmcount_->landmark_graph()->n_landmarks(),
                                    &cuda_open_);

  int n_successors_max = 10 * n_threads_;
  gpu_ram -= 10 * n_threads_ * sizeof(int);
  InitializeGBFSMessage(n_threads_, &m_);
  gpu_ram -= AllocateMessage();
  gpu_ram -= CudaInitializeGBFSMessage(problem_, graph_, n_threads_,
                                       n_successors_max, &cuda_m_);
  int closed_exponent = 13;
  std::size_t closed_size = 1u << closed_exponent;
  gpu_ram -= CudaInitializeClosedList(n_threads_, closed_size, &cuda_closed_);
  gpu_ram -= 2000000000;
  InitCudaSearchGraph(problem_, graph_, closed_exponent, gpu_ram, &cuda_graph_);
}

CudaHetroGBFS::~CudaHetroGBFS() {
  FreeCudaSASPlus(cuda_problem_);
  FreeCudaSuccessorGenerator(cuda_generator_);
  FreeCudaLandmarkGraph(cuda_landmark_graph_);
  FreeCudaZobristHash(cuda_c_hash_);
  FreeCudaZobristHash(cuda_d_hash_);
  FreeCudaSearchGraph(&cuda_graph_);
  CUDA_CHECK(cudaFree(cuda_steps_));
  CUDA_CHECK(cudaFree(cuda_offsets_));
  CudaFreeGBFSMessage(&cuda_m_);

  delete[] m_.h_min;
  delete[] m_.states;
  delete[] m_.accepted;
  FreeGBFSMessage(&m_);

  delete cuda_problem_;
  delete cuda_generator_;
  delete cuda_landmark_graph_;
  delete cuda_c_hash_;
  delete cuda_d_hash_;
}

void CudaHetroGBFS::ClearGPU() {
  CudaNPlanStep<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_.nodes,
                                          cuda_steps_);

  std::vector<int> steps(n_threads_);
  CUDA_CHECK(cudaMemcpy(steps.data(), cuda_steps_, n_threads_ * sizeof(int),
                        cudaMemcpyDeviceToHost));

  std::vector<int> offsets(n_threads_, 0);
  offsets[0] = steps[0];

  for (int i = 1; i < n_threads_; ++i)
    offsets[i] = steps[i] + offsets[i - 1];

  CUDA_CHECK(cudaMemcpy(cuda_offsets_, offsets.data(), n_threads_ * sizeof(int),
                        cudaMemcpyHostToDevice));

  int *cuda_plans = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_plans, offsets.back() * sizeof(int)));

  CudaExtractPlan<<<n_grid_, n_block_>>>(cuda_graph_, cuda_offsets_,
                                         cuda_m_.nodes, cuda_plans);

  std::vector<int> plans(offsets.back());
  std::size_t n_bytes = graph_->n_landmarks_bytes();
  CUDA_CHECK(cudaMemcpy(plans.data(), cuda_plans, offsets.back() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m_.states, cuda_m_.states,
                        n_threads_ * problem_->n_variables() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m_.accepted, cuda_m_.accepted,
                        n_threads_ * n_bytes * sizeof(uint8_t),
                        cudaMemcpyDeviceToHost));

  std::vector<int> state(problem_->n_variables());
  std::vector<int> values(1);
  int h_max = lmcount_->landmark_graph()->n_landmarks() + 1;

  for (int i = 0; i < n_threads_; ++i) {
    int h = m_.h_min[i];
    if (h == -1 || h == h_max) continue;

    memcpy(state.data(), &m_.states[i * problem_->n_variables()],
           problem_->n_variables() * sizeof(int));
    int node = graph_->GenerateNodeIfNotClosed(-1, gpu_start_, state);

    if (node != -1) {
      graph_->SetH(node, h);
      graph_->SetLandmark(node, &m_.accepted[i * n_bytes]);
      values[0] = h;
      int plan_start = i == 0 ? 0 : offsets[i - 1];
      sequences_[node] = std::vector<int>(steps[i]);

      for (int j = 0; j < steps[i]; ++j)
        sequences_[node][j] = plans[plan_start + j];

      open_->Push(values, node, false);

      if (h < best_h_) {
        best_h_ = h;
        best_node_ = node;
        n_plateau_ = 0;
        std::cout << "New best heuristic value: " << best_h_
                  << " (found by GPU)" << std::endl;
        std::cout << "[" << expanded_ << " expanded]" << std::endl;
      }
    }
  }

  CUDA_CHECK(cudaFree(cuda_plans));
}

void CudaHetroGBFS::InitialPushGPU(int node, int h) {
  gpu_start_ = node;

  std::vector<int> state(problem_->n_variables());
  graph_->State(node, state);
  CUDA_CHECK(cudaMemcpy(cuda_m_.states, state.data(),
                        problem_->n_variables() * sizeof(int),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(cuda_m_.accepted, graph_->Landmark(node),
                        graph_->n_landmarks_bytes() * sizeof(uint8_t),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(cuda_m_.h_min, &h, sizeof(int),
                        cudaMemcpyHostToDevice));

  CudaInitialPush<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_, cuda_open_,
                                         n_threads_);
  m_.n_nodes = 1;
  cuda_m_.n_nodes = 1;

  CUDA_CHECK(cudaDeviceSynchronize());
}

void CudaHetroGBFS::InitialEvaluateAndPush() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);
  ++generated_;

  best_h_ = lmcount_->Evaluate(state, nullptr, graph_->Landmark(node));
  graph_->SetH(node, best_h_);
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  best_node_ = node;
  std::vector<int> values{best_h_};
  open_->Push(values, node, false);

  InitialPushGPU(node, best_h_);
}

int CudaHetroGBFS::CpuExpand() {
  thread_local vector<int> state(problem_->n_variables());
  thread_local vector<int> child(problem_->n_variables());
  thread_local vector<int> applicable;
  thread_local vector<int> values(1);

  if (open_->IsEmpty()) return -1;

  int node = open_->Pop();

  if (!graph_->CloseIfNot(node)) return -1;

  ++expanded_;
  graph_->Expand(node, state);

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) return -1;

  for (auto o : applicable) {
    problem_->ApplyEffect(o, state, child);

    int child_node = graph_->GenerateNodeIfNotClosed(o, node, state, child);
    if (child_node == -1) continue;
    ++generated_;

    int h = lmcount_->Evaluate(child, graph_->Landmark(node),
                               graph_->Landmark(child_node));

    if (h == -1) continue;

    graph_->SetH(child_node, h);
    values[0] = h;
    open_->Push(values, child_node, false);
    ++n_plateau_;

    if (h < best_h_) {
      best_h_ = h;
      best_node_ = child_node;
      n_plateau_ = 0;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
      std::cout << "[" << expanded_ << " expanded]" << std::endl;
    }
  }

  return -1;
}

int CudaHetroGBFS::Search() {
  InitialEvaluateAndPush();
  m_.n_nodes += 1;
  cuda_m_.n_nodes += 1;

  int goal = -1;
  int *cuda_goal = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_goal, sizeof(int)));
  CUDA_CHECK(cudaMemcpy(cuda_goal, &goal, sizeof(int), cudaMemcpyHostToDevice));

  while (goal == -1) {
    cudaEvent_t pop_fin;
    CUDA_CHECK(cudaEventCreate(&pop_fin));
    CudaPop<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_, cuda_open_,
                                   cuda_closed_);
    CUDA_CHECK(cudaEventRecord(pop_fin, 0));

    while (cudaEventQuery(pop_fin) == cudaErrorNotReady) {
      goal = CpuExpand();

      if (goal != -1) {
        CUDA_CHECK(cudaFree(cuda_goal));

        return goal;
      }
    }

    expanded_ += PrepareExpansion(n_threads_, &m_, &cuda_m_);

    cudaEvent_t expand_fin;
    CUDA_CHECK(cudaEventCreate(&expand_fin));
    CudaExpand<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_, n_threads_,
                                      cuda_goal);
    CUDA_CHECK(cudaEventRecord(expand_fin, 0));

    while (cudaEventQuery(expand_fin) == cudaErrorNotReady) {
      goal = CpuExpand();

      if (goal != -1) {
        CUDA_CHECK(cudaFree(cuda_goal));

        return goal;
      }
    }

    CUDA_CHECK(cudaMemcpy(&goal, cuda_goal, sizeof(int),
                          cudaMemcpyDeviceToHost));

    if (goal != -1) {
      std::cout << "goal found by GPU" << std::endl;
      goal_on_gpu_ = true;
      break;
    }

    generated_ += PrepareSort(n_threads_, &m_, &cuda_m_);

    CUDA_CHECK(cudaMemcpy(m_.h_min, cuda_m_.h_min, n_threads_ * sizeof(int),
                          cudaMemcpyDeviceToHost));

    int h_min = -1;

    for (int i = 0; i < n_threads_; ++i) {
      if (m_.h_min[i] != -1 && (h_min == -1 || m_.h_min[i] < h_min))
        h_min = m_.h_min[i];
    }

    if (m_.n_nodes >= static_cast<int>(cuda_graph_.node_max - 1)
        || h_min < best_h_) {
      std::cout << "clear GPU RAM" << std::endl;
      ClearGPU();
      CudaClearOpenList(n_threads_, &cuda_open_);
      CudaClearClosedList(n_threads_, &cuda_closed_);
      m_.n_nodes = 0;
      cuda_m_.n_nodes = 0;
      InitialPushGPU(best_node_, best_h_);
      continue;
    } else if (gpu_threshold_ > 0 && n_plateau_ > gpu_threshold_) {
      std::cout << "clear GPU RAM" << std::endl;
      ClearGPU();

      if (gpu_threshold_ > 0 && n_plateau_ > gpu_threshold_) {
        CudaClearOpenList(n_threads_, &cuda_open_);
        CudaClearClosedList(n_threads_, &cuda_closed_);
        m_.n_nodes = 0;
        cuda_m_.n_nodes = 0;
        InitialPushGPU(best_node_, best_h_);
        n_plateau_ = 0;
        continue;
      }
    }

    cudaEvent_t sort_fin;
    CUDA_CHECK(cudaEventCreate(&sort_fin));
    CudaSortChildren<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_);
    CUDA_CHECK(cudaEventRecord(sort_fin, 0));

    while(cudaEventQuery(sort_fin) == cudaErrorNotReady){
      goal = CpuExpand();

      if (goal != -1) {
        CUDA_CHECK(cudaFree(cuda_goal));

        return goal;
      }
    }

    cudaEvent_t push_fin;
    CUDA_CHECK(cudaEventCreate(&push_fin));
    CudaPush<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_, cuda_closed_,
                                    cuda_open_);
    CUDA_CHECK(cudaEventRecord(push_fin, 0));

    while(cudaEventQuery(push_fin) == cudaErrorNotReady){
      goal = CpuExpand();

      if (goal != -1) {
        CUDA_CHECK(cudaFree(cuda_goal));

        return goal;
      }
    }
  }

  CUDA_CHECK(cudaFree(cuda_goal));

  return goal;
}

std::vector<int> CudaHetroGBFS::ExtractPlan(int node) {
  if (node == -1) return std::vector<int>{-1};

  int current = goal_on_gpu_ ? gpu_start_ : node;

  vector<int> result;

  while (graph_->Parent(current) != -1) {
    if (graph_->Action(current) == -1) {
      result.insert(result.begin(), sequences_[current].begin(),
                    sequences_[current].end());
    } else {
      result.insert(result.begin(), graph_->Action(current));
    }

    current = graph_->Parent(current);
  }

  if (!goal_on_gpu_) return result;

  int *goals = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&goals, sizeof(int)));
  CUDA_CHECK(cudaMemcpy(goals, &node, sizeof(int), cudaMemcpyHostToDevice));

  CudaNPlanStep<<<1, 1>>>(cuda_graph_, goals, cuda_steps_);
  int step;
  CUDA_CHECK(cudaMemcpy(&step, cuda_steps_, sizeof(int),
                        cudaMemcpyDeviceToHost));

  int *cuda_plan = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_plan, step * sizeof(int)));

  CudaExtractPlan<<<1, 1>>>(cuda_graph_, cuda_steps_, goals, cuda_plan);

  int *plan = new int[step];
  CUDA_CHECK(cudaMemcpy(plan, cuda_plan, step * sizeof(int),
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < step; ++i)
    result.push_back(plan[i]);

  delete[] plan;
  CUDA_CHECK(cudaFree(cuda_plan));

  return result;
}

void CudaHetroGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
}

} // namespace pplanner
