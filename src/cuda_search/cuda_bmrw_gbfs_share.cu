#include "cuda_search/cuda_bmrw_gbfs_share.cuh"

#include <array>

#include "evaluator.h"
#include "open_list_factory.h"
#include "cuda_common/cuda_check.cuh"
#include "cuda_search/cuda_random_walk.cuh"

namespace pplanner {

using std::vector;

extern __constant__ CudaSASPlus cuda_problem;
extern __constant__ CudaSuccessorGenerator cuda_generator;
extern __constant__ CudaLandmarkGraph cuda_landmark_graph;

void CudaBMRWGBFSShare::Init(const boost::property_tree::ptree &pt) {
  if (auto opt = pt.get_optional<int>("n_grid"))
    n_grid_ = opt.get();

  if (auto opt = pt.get_optional<int>("n_block"))
    n_block_ = opt.get();

  n_threads_ = n_grid_ * n_block_;

  if (auto opt = pt.get_optional<int>("n_elite"))
    n_elite_ = opt.get();

  if (auto opt = pt.get_optional<int>("walk_length"))
    walk_length_ = opt.get();

  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  graph_ = std::make_shared<SearchGraphWithLandmarks>(
      problem_, closed_exponent);

  lmcount_ = std::make_shared<LandmarkCountBase>(
      problem_, false, false, false, false);

  landmark_id_max_ = lmcount_->landmark_graph()->landmark_id_max();
  n_landmark_bytes_ = (landmark_id_max_ + 7) / 8;
  graph_->InitLandmarks(lmcount_->landmark_graph());

  std::vector<std::shared_ptr<Evaluator> > evaluators;

  auto open_option = pt.get_child("open_list");
  open_ = OpenListFactory(open_option, evaluators);
  rw_open_ = OpenListFactory(open_option, evaluators);

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram"))
    ram = opt.get();

  std::size_t size = graph_->ReserveByRAMSize(ram);
  sequences_.reserve(size);

  InitCudaSASPlus(problem_, cuda_problem_);
  InitCudaSuccessorGenerator(generator_, cuda_generator_);
  InitCudaLandmarkGraph(lmcount_->landmark_graph(), cuda_landmark_graph_);

  InitRandomWalkMessage(n_grid_, n_block_, walk_length_,
                        problem_->n_variables(), landmark_id_max_, &m_);
  CudaInitRandomWalkMessage(n_grid_, n_block_, walk_length_,
                            problem_->n_variables(), landmark_id_max_,
                            &cuda_m_);
  UploadStatistics(m_, n_threads_, &cuda_m_);
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_problem, cuda_problem_,
                                sizeof(CudaSASPlus)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_generator, cuda_generator_,
                                sizeof(CudaSuccessorGenerator)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_landmark_graph, cuda_landmark_graph_,
                                sizeof(CudaLandmarkGraph)));

  for (int i = 0; i < n_threads_; ++i)
    m_.first_eval[i] = false;
}

bool CudaBMRWGBFSShare::PopStates(vector<int> &parents) {
  int offset = 0;
  int i = 0;

  while (i < n_threads_) {
    int node = -1;
    int h = -1;

    if (i == 0 && rw_open_->IsEmpty()) {
      return false;
    } else if (rw_open_->IsEmpty()) {
      if (offset == 0) offset = i;
      h = m_.best_h[(i - offset) % offset];
      node = parents[(i - offset) % offset];
    } else {
      h = rw_open_->MinimumValue(0);
      node = rw_open_->Pop();
    }

    if (graph_->Action(node) != -1) {
      m_.first_eval[i] = true;
      memcpy(&m_.best_accepted[i * n_landmark_bytes_],
             graph_->ParentLandmark(node),
             n_landmark_bytes_ * sizeof(uint8_t));
    } else {
      m_.first_eval[i] = false;
      memcpy(&m_.best_accepted[i * n_landmark_bytes_],
             graph_->Landmark(node),
             n_landmark_bytes_ * sizeof(uint8_t));
    }

    m_.best_h[i] = h;
    graph_->State(node, &m_.best_states[i * problem_->n_variables()]);
    parents[i] = node;
    ++i;
  }

  Upload(m_, n_threads_, problem_->n_variables(), n_landmark_bytes_, &cuda_m_);

  return true;
}

void CudaBMRWGBFSShare::GenerateChildren(int parent, vector<int> &values,
                                         const vector<int> &state) {
  thread_local vector<int> child;
  thread_local vector<int> applicable;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    ++dead_ends_;

    return;
  }

  for (auto o : applicable) {
    child = state;
    problem_->ApplyEffect(o, child);
    int node = graph_->GenerateNode(o, parent, state, child);
    ++generated_;
    rw_open_->Push(values, node, false);
  }
}

int CudaBMRWGBFSShare::PushStates(const vector<int> &parents, vector<int> &arg_h) {
  thread_local std::vector<int> values(1);
  thread_local std::vector<int> state(problem_->n_variables());

  Download(cuda_m_, n_threads_, problem_->n_variables(), n_landmark_bytes_,
           walk_length_, &m_);

  if (n_elite_ > 0 && n_elite_ < n_threads_) {
    std::iota(arg_h.begin(), arg_h.end(), 0);
    auto cond = [this](int x, int y) {
      return this->m_.best_h[x] < this->m_.best_h[y];
    };
    std::sort(arg_h.begin(), arg_h.end(), cond);
  }

  int counter = 0;

  for (auto i : arg_h) {
    int h = m_.best_h[i];

    if (h == -1) continue;

    memcpy(state.data(), &m_.best_states[i * problem_->n_variables()],
           problem_->n_variables() * sizeof(int));
    int node = graph_->GenerateAndCloseNode(-1, parents[i], state);

    if (node == -1) continue;

    ++generated_;
    graph_->SetLandmark(node, &m_.best_accepted[i * n_landmark_bytes_]);
    sequences_.resize(node + 1);
    int length = m_.best_length[i];
    sequences_[node].resize(length);
    memcpy(sequences_[node].data(), &m_.best_sequences[i * walk_length_],
           length * sizeof(int));
    ++plateau_;

    if (h < best_h_) {
      best_h_ = h;
      plateau_ = 0;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
    }

    if (h == 0) return node;

    values[0] = h;

    if (counter < n_elite_) {
      GenerateChildren(node, values, state);
      ++counter;
    } else {
      rw_open_->Push(values, node, false);
    }
  }

  return -1;
}

void CudaBMRWGBFSShare::Restart() {
  std::cout << "restart" << std::endl;
  graph_->Clear();
  sequences_.clear();
  InitialEvaluate();
}

void CudaBMRWGBFSShare::InitialEvaluate() {
  auto state = problem_->initial();
  int node = graph_->GenerateAndCloseNode(-1, -1, state);
  ++generated_;

  best_h_ = lmcount_->Evaluate(state, nullptr, graph_->Landmark(node));
  graph_->SetH(node, best_h_);
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  ++evaluated_;

  std::vector<int> values{best_h_};
  open_->Push(values, node, false);
  GenerateChildren(node, values, state);
}

int CudaBMRWGBFSShare::CpuExpand() {
  thread_local vector<int> state(problem_->n_variables());
  thread_local vector<int> child(problem_->n_variables());
  thread_local vector<int> applicable;
  thread_local vector<int> values(1);

  if (open_->IsEmpty()) return -1;

  int node = open_->Pop();

  ++expanded_;
  graph_->Expand(node, state);

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    ++dead_ends_;
    return -1;
  }

  for (auto o : applicable) {
    child = state;
    problem_->ApplyEffect(o, child);

    int child_node = graph_->GenerateAndCloseNode(o, node, state, child);
    if (child_node == -1) continue;
    ++generated_;

    int h = lmcount_->Evaluate(child, graph_->Landmark(node),
                               graph_->Landmark(child_node));
    ++evaluated_;

    if (h == -1) {
      ++dead_ends_;
      continue;
    }

    graph_->SetH(child_node, h);
    values[0] = h;
    open_->Push(values, child_node, false);
    rw_open_->Push(values, child_node, false);
    ++plateau_;

    if (h < best_h_) {
      best_h_ = h;
      plateau_ = 0;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, "
                << expanded_ << " expanded]" << std::endl;
    }
  }

  return -1;
}

vector<int> CudaBMRWGBFSShare::Plan() {
  std::vector<int> parents(n_threads_);
  std::vector<int> arg_h(n_threads_);
  InitialEvaluate();

  while (true) {
    if (open_->IsEmpty() && rw_open_->IsEmpty()) Restart();

    int goal = CpuExpand();

    if (goal != -1) return ExtractPlan(goal);

    if (!PopStates(parents)) continue;

    cudaEvent_t fin;
    CUDA_CHECK(cudaEventCreate(&fin));
    RandomWalk<<<n_grid_, n_block_>>>(walk_length_, cuda_m_);
    CUDA_CHECK(cudaEventRecord(fin, 0));

    while(cudaEventQuery(fin) == cudaErrorNotReady){
      goal = CpuExpand();

      if (goal != -1) return ExtractPlan(goal);
    }

    goal = PushStates(parents, arg_h);

    if (goal != -1) return ExtractPlan(goal);
  }
}

std::vector<int> CudaBMRWGBFSShare::ExtractPlan(
    int node) {
  DownloadStatistics(cuda_m_, n_threads_, &m_);

  for (int i = 0; i < n_threads_; ++i) {
    generated_ += m_.generated[i];
    expanded_ += m_.expanded[i];
    evaluated_ += m_.evaluated[i];
    dead_ends_ += m_.dead_ends[i];
  }

  if (node == -1) return std::vector<int>{-1};

  vector<int> result;

  while (graph_->Parent(node) != -1) {
    if (graph_->Action(node) == -1) {
      result.insert(result.begin(), sequences_[node].begin(),
                    sequences_[node].end());
    } else {
      result.insert(result.begin(), graph_->Action(node));
    }

    node = graph_->Parent(node);
  }

  return result;
}

void CudaBMRWGBFSShare::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

CudaBMRWGBFSShare::~CudaBMRWGBFSShare() {
  FreeCudaSASPlus(cuda_problem_);
  FreeCudaSuccessorGenerator(cuda_generator_);
  FreeCudaLandmarkGraph(cuda_landmark_graph_);
  FreeRandomWalkMessage(&m_);
  CudaFreeRandomWalkMessage(&cuda_m_);
  delete cuda_problem_;
  delete cuda_generator_;
  delete cuda_landmark_graph_;
}

} // namespace pplanner
