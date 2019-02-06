#include "cuda_search/cuda_bmrw_gbfs_focus2.cuh"

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

void CudaBMRWGBFSFocus2::Init(const boost::property_tree::ptree &pt) {
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
  rw_graph_ = std::make_shared<SearchGraphWithLandmarks>(
      problem_, closed_exponent);

  lmcount_ = std::make_shared<LandmarkCountBase>(
      problem_, false, false, false, false);

  landmark_id_max_ = lmcount_->landmark_graph()->landmark_id_max();
  n_landmark_bytes_ = (landmark_id_max_ + 7) / 8;
  graph_->InitLandmarks(lmcount_->landmark_graph());
  rw_graph_->InitLandmarks(lmcount_->landmark_graph());

  open_option_ = pt.get_child("open_list");
  rw_open_ = SharedOpenListFactory(open_option_, evaluators_);

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram"))
    ram = opt.get();

  std::size_t gbfs_ram = static_cast<std::size_t>(
      static_cast<float>(ram) * 0.7);
  std::size_t size = graph_->ReserveByRAMSize(gbfs_ram);
  std::size_t rw_ram = static_cast<std::size_t>(static_cast<float>(ram) * 0.3);
  std::size_t rw_size = rw_graph_->ReserveByRAMSize(rw_ram);
  sequences_.reserve(rw_size);

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

void CudaBMRWGBFSFocus2::RWInitialEvaluate() {
  auto state = problem_->initial();
  int node = rw_graph_->GenerateAndCloseNode(-1, -1, state);
  ++generated_;

  int h = lmcount_->Evaluate(state, nullptr, rw_graph_->Landmark(node));
  rw_graph_->SetH(node, h);
  ++evaluated_;

  std::vector<int> values{h};
  GenerateChildren(0, values, state);
}

void CudaBMRWGBFSFocus2::PopStates(vector<int> &parents) {
  int offset = 0;

  for (int i = 0; i < n_threads_; ++i) {
    int node = -1;
    int h = -1;

    if (rw_open_->IsEmpty()) {
      if (offset == 0) offset = i;
      h = m_.best_h[(i - offset) % offset];
      node = parents[(i - offset) % offset];
    } else {
      h = rw_open_->MinimumValue(0);
      node = rw_open_->Pop();
    }

    if (rw_graph_->Action(node) != -1) {
      m_.first_eval[i] = true;
      memcpy(&m_.best_accepted[i * n_landmark_bytes_],
             rw_graph_->ParentLandmark(node),
             n_landmark_bytes_ * sizeof(uint8_t));
    } else {
      m_.first_eval[i] = false;
      memcpy(&m_.best_accepted[i * n_landmark_bytes_],
             rw_graph_->Landmark(node),
             n_landmark_bytes_ * sizeof(uint8_t));
    }

    m_.best_h[i] = h;
    rw_graph_->State(node, &m_.best_states[i * problem_->n_variables()]);
    parents[i] = node;
  }

  Upload(m_, n_threads_, problem_->n_variables(), n_landmark_bytes_, &cuda_m_);
}


void CudaBMRWGBFSFocus2::GenerateChildren(int parent, vector<int> &values,
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
    int node = rw_graph_->GenerateNode(o, parent, state, child);
    ++generated_;
    rw_open_->Push(values, node, false);
  }
}

int CudaBMRWGBFSFocus2::PushStates(const vector<int> &parents, vector<int> &arg_h) {
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
    int node = rw_graph_->GenerateAndCloseNode(-1, parents[i], state);

    if (node == -1) continue;

    ++generated_;
    rw_graph_->SetLandmark(node, &m_.best_accepted[i * n_landmark_bytes_]);
    sequences_.resize(node + 1);
    int length = m_.best_length[i];
    sequences_[node].resize(length);
    memcpy(sequences_[node].data(), &m_.best_sequences[i * walk_length_],
           length * sizeof(int));

    values[0] = h;

    if (h < best_h_) {
      best_h_ = h;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
      int gbfs_node = graph_->GenerateNodeIfNotClosed(-1, -1, state);
      graph_->SetLandmark(gbfs_node, &m_.best_accepted[i * n_landmark_bytes_]);
      opens_.push_back(SharedOpenListFactory(open_option_, evaluators_));
      opens_.back()->Push(values, gbfs_node, true);
      gbfs_node_to_rw_node_[gbfs_node] = node;
      std::cout << "new focus" << std::endl;
    }

    if (h == 0) return node;

    if (counter < n_elite_) {
      GenerateChildren(node, values, state);
      ++counter;
    } else {
      rw_open_->Push(values, node, false);
    }
  }

  return -1;
}

void CudaBMRWGBFSFocus2::Restart() {
  std::cout << "restart" << std::endl;
  rw_graph_->ClearClosed();
  RWInitialEvaluate();
}

void CudaBMRWGBFSFocus2::InitialEvaluate() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);
  ++generated_;

  best_h_ = lmcount_->Evaluate(state, nullptr, graph_->Landmark(node));
  graph_->SetH(node, best_h_);
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  ++evaluated_;

  std::vector<int> values{best_h_};
  opens_.push_back(SharedOpenListFactory(open_option_, evaluators_));
  opens_[0]->Push(values, node, false);
}

int CudaBMRWGBFSFocus2::GreedyOpen() {
  int arg_min = -1;

  auto iter = std::remove_if(opens_.begin(), opens_.end(),
                             [](std::shared_ptr<OpenList> p)->bool
                             { return p->IsEmpty(); });
  opens_.erase(iter, opens_.end());
  std::vector<int> minimum_values;

  for (int i = 0, n = opens_.size(); i < n; ++i) {
    if (opens_[i]->IsEmpty()) continue;

    if (arg_min == -1 || opens_[i]->MinimumValues() < minimum_values) {
      minimum_values = opens_[i]->MinimumValues();
      arg_min = i;
    }
  }

  return arg_min;
}

int CudaBMRWGBFSFocus2::CpuExpand() {
  thread_local vector<int> state(problem_->n_variables());
  thread_local vector<int> child(problem_->n_variables());
  thread_local vector<int> applicable;
  thread_local vector<int> values(1);

  int arg_open = GreedyOpen();

  if (arg_open == -1) return -1;

  int node = opens_[arg_open]->Pop();

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
    opens_[arg_open]->Push(values, child_node, false);

    //if (rw_open_->IsEmpty() || h < rw_open_->MinimumValue(0)) {
    //  int rw_node = rw_graph_->GenerateNodeIfNotClosed(-1, -1, child);
    //  rw_graph_->SetLandmark(rw_node, graph_->Landmark(child_node));
    //  rw_open_->Push(values, rw_node, true);
    //  rw_node_to_gbfs_node_[rw_node] = child_node;
    //  std::cout << "node inserted into BMRW open list" << std::endl;
    //}

    if (h < best_h_) {
      best_h_ = h;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, "
                << expanded_ << " expanded]" << std::endl;
    }
  }

  return -1;
}

vector<int> CudaBMRWGBFSFocus2::Plan() {
  std::vector<int> parents(n_threads_);
  std::vector<int> arg_h(n_threads_);
  InitialEvaluate();
  RWInitialEvaluate();

  while (!opens_.empty()) {
    if (rw_open_->IsEmpty()) Restart();

    PopStates(parents);

    cudaEvent_t fin;
    CUDA_CHECK(cudaEventCreate(&fin));
    RandomWalk<<<n_grid_, n_block_>>>(walk_length_, cuda_m_);
    CUDA_CHECK(cudaEventRecord(fin, 0));

    while(cudaEventQuery(fin) == cudaErrorNotReady){
      int goal = CpuExpand();

      if (goal != -1) return ExtractPlan(goal, false);
    }

    int goal = PushStates(parents, arg_h);

    if (goal != -1) return ExtractPlan(goal, true);
  }

  return ExtractPlan(-1, true);
}

std::vector<int> CudaBMRWGBFSFocus2::ExtractPlanFromRWGraph(int node) {
  vector<int> result;

  while (true) {
    if (rw_graph_->Parent(node) == -1) {
      if (rw_node_to_gbfs_node_.find(node) == rw_node_to_gbfs_node_.end())
        break;

      std::cout << "BMRW searched a state found by GBFS" << std::endl;
      int gbfs_node = rw_node_to_gbfs_node_[node];
      auto gbfs_result = ExtractPlanFromGBFS(gbfs_node);
      gbfs_result.insert(gbfs_result.end(), result.begin(), result.end());

      return gbfs_result;
    }

    if (rw_graph_->Action(node) == -1) {
      result.insert(result.begin(), sequences_[node].begin(),
                    sequences_[node].end());
    } else {
      result.insert(result.begin(), rw_graph_->Action(node));
    }

    node = rw_graph_->Parent(node);
  }

  return result;
}

std::vector<int> CudaBMRWGBFSFocus2::ExtractPlanFromGBFS(int node) {
  vector<int> result;

  while (node != 0) {
    if (graph_->Action(node) == -1) {
      std::cout << "GBFS searched a state found by BMRW" << std::endl;
      int rw_node = gbfs_node_to_rw_node_[node];
      auto rw_result = ExtractPlanFromRWGraph(rw_node);
      rw_result.insert(rw_result.end(), result.begin(), result.end());

      return rw_result;
    } else {
      result.insert(result.begin(), graph_->Action(node));
    }

    node = graph_->Parent(node);
  }

  return result;
}

std::vector<int> CudaBMRWGBFSFocus2::ExtractPlan(int node, bool from_rw) {
  DownloadStatistics(cuda_m_, n_threads_, &m_);

  for (int i = 0; i < n_threads_; ++i) {
    generated_ += m_.generated[i];
    expanded_ += m_.expanded[i];
    evaluated_ += m_.evaluated[i];
    dead_ends_ += m_.dead_ends[i];
  }

  if (node == -1) return std::vector<int>{-1};

  if (from_rw) return ExtractPlanFromRWGraph(node);

  return ExtractPlanFromGBFS(node);
}

void CudaBMRWGBFSFocus2::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

CudaBMRWGBFSFocus2::~CudaBMRWGBFSFocus2() {
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
