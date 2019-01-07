#include "cuda_search/cuda_rwagbfs.cuh"

#include "evaluator.h"
#include "open_list_factory.h"
#include "cuda_common/cuda_check.cuh"
#include "cuda_search/cuda_random_walk.cuh"

namespace pplanner {

using std::vector;

extern __constant__ CudaSASPlus cuda_problem;
extern __constant__ CudaSuccessorGenerator cuda_generator;
extern __constant__ CudaLandmarkGraph cuda_landmark_graph;

void CudaRWAGBFS::Init(const boost::property_tree::ptree &pt) {
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

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option, evaluators);

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

void CudaRWAGBFS::InitialEvaluate() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);
  ++generated_;

  best_h_ = lmcount_->Evaluate(state, nullptr, graph_->Landmark(node));
  graph_->SetH(node, best_h_);
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  ++evaluated_;

  std::vector<int> values{best_h_};
  open_list_->Push(values, node, false);
}

int CudaRWAGBFS::PushStates(const vector<int> &parents) {
  thread_local vector<int> state(problem_->n_variables());
  thread_local vector<int> values(1);

  for (int i = 0; i < n_threads_; ++i) {
    int h = m_.best_h[i];

    if (h == -1) continue;

    memcpy(state.data(), &m_.best_states[i * problem_->n_variables()],
           problem_->n_variables() * sizeof(int));
    int node = graph_->GenerateNodeIfNotClosed(-1, parents[i], state);

    if (node == -1) continue;

    ++generated_;
    graph_->SetH(node, h);
    graph_->SetLandmark(node, &m_.best_accepted[i * n_landmark_bytes_]);
    sequences_.resize(node + 1);
    int length = m_.best_length[i];
    sequences_[node].resize(length);
    memcpy(sequences_[node].data(), &m_.best_sequences[i * walk_length_],
           length * sizeof(int));

    if (h < best_h_) {
      best_h_ = h;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
    }

    if (h == 0) return node;

    values[0] = h;
    open_list_->Push(values, node, false);
  }

  return -1;
}

int CudaRWAGBFS::Search() {
  std::vector<int> parents(n_threads_);
  std::vector<int> arg_h(n_threads_);
  InitialEvaluate();
  int counter = 0;

  while (!open_list_->IsEmpty()) {
    if (counter < n_threads_) {
      int goal = Expand(parents, &counter);

      if (goal != -1) return goal;
    }

    if (counter >= n_threads_) {
      counter = 0;
      Upload(m_, n_threads_, problem_->n_variables(), n_landmark_bytes_,
             &cuda_m_);
      cudaEvent_t fin;
      CUDA_CHECK(cudaEventCreate(&fin));
      RandomWalk<<<n_grid_, n_block_>>>(walk_length_, cuda_m_);
      CUDA_CHECK(cudaEventRecord(fin, 0));

      while(cudaEventQuery(fin) == cudaErrorNotReady){
        int goal = Expand(parents, &counter);

        if (goal != -1) return goal;
      }

      Download(cuda_m_, n_threads_, problem_->n_variables(), n_landmark_bytes_,
               walk_length_, &m_);

      int goal = PushStates(parents);

      if (goal != -1) return goal;
    }
  }

  return -1;
}

int CudaRWAGBFS::Expand(vector<int> &parents, int *counter) {
  thread_local vector<int> state(problem_->n_variables());
  thread_local vector<int> child(problem_->n_variables());
  thread_local vector<int> applicable;
  thread_local vector<int> values(1);

  int node = open_list_->Pop();

  if (!graph_->CloseIfNot(node)) return -1;

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

    int child_node = graph_->GenerateNodeIfNotClosed(o, node, state, child);
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
    open_list_->Push(values, child_node, false);

    int id = *counter % n_threads_;
    m_.best_h[id] = h;
    memcpy(&m_.best_states[id * problem_->n_variables()], child.data(),
           problem_->n_variables() * sizeof(int));
    memcpy(&m_.best_accepted[id * n_landmark_bytes_],
           graph_->Landmark(child_node), n_landmark_bytes_ * sizeof(uint8_t));
    parents[id] = child_node;
    *counter = *counter + 1;

    if (h < best_h_) {
      best_h_ = h;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, "
                << expanded_ << " expanded]" << std::endl;
    }
  }

  return -1;
}
std::vector<int> CudaRWAGBFS::ExtractPlan(int node) {
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

void CudaRWAGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

CudaRWAGBFS::~CudaRWAGBFS() {
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
