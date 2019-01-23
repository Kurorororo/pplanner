#include "mpi_cuda_search/hetro_hdgbfs_lg.cuh"

#include <algorithm>
#include <iostream>
#include <unordered_set>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"
#include "distributed_search_graph_factory.h"
#include "hash/distribution_hash_factory.h"

namespace pplanner {

using std::make_shared;
using std::size_t;
using std::unordered_set;
using std::vector;

std::size_t HetroHDGBFSLG::AllocateMessage() {
  m_.h_min = new int[n_threads_];
  m_.states = new int[n_threads_ * problem_->n_variables()];
  m_.accepted = new uint8_t[n_threads_ * graph_->n_landmarks_bytes()];

  CUDA_CHECK(cudaMalloc((void**)&cuda_steps_, n_threads_ * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&cuda_offsets_, n_threads_ * sizeof(int)));

  return 2 * n_threads_ * sizeof(int);
}

void HetroHDGBFSLG::InitGPU(const boost::property_tree::ptree &pt) {
  if (auto opt = pt.get_optional<int>("n_grid"))
    n_grid_ = opt.get();

  if (auto opt = pt.get_optional<int>("n_block"))
    n_block_ = opt.get();

  n_threads_ = n_grid_ * n_block_;

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

void HetroHDGBFSLG::Init(const boost::property_tree::ptree &pt) {
  MPI_Comm_rank(MPI_COMM_WORLD, &rank_);

  graph_->InitLandmarks(lmcount_->landmark_graph());

  if (auto opt = pt.get_optional<int>("take"))
    take_= opt.get();

  std::vector<std::shared_ptr<Evaluator> > evaluators;
  auto open_list_option = pt.get_child("open_list");
  open_ = OpenListFactory(open_list_option, evaluators);

  if (auto opt = pt.get_optional<int>("local_open")) {
    use_local_open_ = true;
    local_open_list_ = OpenListFactory(open_list_option, evaluators);
  }

  if (auto opt = pt.get_optional<int>("reset_best"))
    reset_best_ = true;

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram"))
    ram = opt.get();

  graph_->ReserveByRAMSize(ram);

  MPI_Comm_size(MPI_COMM_WORLD, &world_size_);

  unsigned int buffer_size = 1000000000;
  mpi_buffer_ = new unsigned char[buffer_size];
  MPI_Buffer_attach((void*)mpi_buffer_, buffer_size);

  outgoing_buffers_.resize(world_size_);

  if (rank_ == 0) InitGPU(pt);
}

CudaHetroGBFS::~CudaHetroGBFS() {
  if (rank_ == 0) {
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

  Flush(kNodeTag);
  int detach_size;
  MPI_Buffer_detach(&mpi_buffer_, &detach_size);
  delete[] mpi_buffer_;
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
  CUDA_CHECK(cudaMemcpy(m_.h_min, cuda_m_.h_min, n_threads_ * sizeof(int),
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
        std::cout << "New best heuristic value: " << best_h_
                  << " (found by GPU)" << std::endl;
        std::cout << "[" << expanded_ << " expanded]" << std::endl;
      }
    }
  }

  CudaClearOpenList(n_threads_, &cuda_open_);
  CudaClearClosedList(n_threads_, &cuda_closed_);
  m_.n_nodes = 0;
  cuda_m_.n_nodes = 0;
  CUDA_CHECK(cudaFree(cuda_plans));
}

void HetroHDGBFSLG::BufferNode(int to_rank, int h, int action, int parent_node,
                               uint32_t hash_value, const uint32_t *packed,
                               const uint8_t *accepted) {
  unsigned char *buffer = ExtendOutgoingBuffer(
      to_rank, sizeof(int) + node_size());
  int info[4];
  info[0] = h;
  info[1] = action;
  info[2] = parent_node;
  info[3] = rank_;
  memcpy(buffer, info, 4 * sizeof(int));
  memcpy(buffer + 4 * sizeof(int), &hash_value, sizeof(uint32_t));
  memcpy(buffer + 4 * sizeof(int) + sizeof(uint32_t), packed,
         graph_->state_size());
  memcpy(buffer + 4 * sizeof(int) + 2 * sizeof(uint32_t), accepted,
         graph_->n_landmarks_bytes() * sizeof(uint8_t));
}

int HetroHDGBFSLG::CpuLoop() {
}


int HetroHDGBFSLG::HetroSearch() {
  int goal = -1;
  int *cuda_goal = nullptr;

  CUDA_CHECK(cudaMalloc((void**)&cuda_goal, sizeof(int)));
  CUDA_CHECK(cudaMemcpy(cuda_goal, &goal, sizeof(int),
                        cudaMemcpyHostToDevice));

  while (!ReceiveTermination()) {
    if (best_node_ == -1) {
      goal = CpuExpand();

      if (goal != -1) {
        CUDA_CHECK(cudaFree(cuda_goal));
        SendTermination();

        return goal;
      }

      continue;
    }

    if (m_.n_nodes == 0)
      InitialPushGPU(best_node_, best_h_);

    cudaEvent_t pop_fin;
    CUDA_CHECK(cudaEventCreate(&pop_fin));
    CudaPop<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_, cuda_open_,
                                   cuda_closed_);
    CUDA_CHECK(cudaEventRecord(pop_fin, 0));

    while (cudaEventQuery(pop_fin) == cudaErrorNotReady) {
      goal = CpuExpand();

      if (goal != -1) {
        CUDA_CHECK(cudaFree(cuda_goal));
        SendTermination();

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
        SendTermination();

        return goal;
      }
    }

    CUDA_CHECK(cudaMemcpy(&goal, cuda_goal, sizeof(int),
                          cudaMemcpyDeviceToHost));

    if (goal != -1) {
      std::cout << "goal found by GPU" << std::endl;
      goal_on_gpu_ = true;
      SendTermination();
      break;
    }

    generated_ += PrepareSort(n_threads_, &m_, &cuda_m_);

    if (m_.n_nodes >= static_cast<int>(cuda_graph_.node_max - 1)) {
      std::cout << "clear GPU RAM" << std::endl;
      ClearGPU();
      continue;
    }

    cudaEvent_t sort_fin;
    CUDA_CHECK(cudaEventCreate(&sort_fin));
    CudaSortChildren<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_);
    CUDA_CHECK(cudaEventRecord(sort_fin, 0));

    while(cudaEventQuery(sort_fin) == cudaErrorNotReady){
      goal = CpuExpand();

      if (goal != -1) {
        CUDA_CHECK(cudaFree(cuda_goal));
        SendTermination();

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
        SendTermination();

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
  }

  CUDA_CHECK(cudaFree(cuda_goal));

  return -1;
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

void HetroHDGBFSLG::InitialEvaluateAndPush() {
  auto state = problem_->initial();
  uint32_t world_size = static_cast<uint32_t>(world_size_);
  initial_rank_ = z_hash_->operator()(state) % world_size;

  if (rank_ == initial_rank_) {
    int node = -1;
    node = graph_->GenerateNode(-1, -1, state, -1);
    IncrementGenerated();
    int h = Evaluate(state, node, -1, best_values_);
    graph_->SetH(node, h);

    best_h_ = h;
    best_node_ = node;
    local_best_h_ = h;
    local_node_ = node;

    std::cout << "Initial heuristic value: " << h << std::endl;
  }
}

int HetroHDGBFSLG::CpuExpand() {
  thread_local vector<int> state(problem_->n_variables());
  thread_local vector<int> applicable;
  thread_local vector<vector<int> > state_array;
  thread_local vector<vector<uint32_t> > packed_array;
  thread_local vector<int> action_array;
  thread_local vector<uint32_t> hash_array;
  thread_local vector<int> h_array;
  thread_local vector<vector<uint8_t>> accepted_arary;

  ReceiveNodes();

  if (NoNode()) return -1;

  int node = local_node_ == -1 ? open_->Pop() : local_node_;
  bool from_local = local_node_ != -1;
  local_node_ = -1;

  if (!graph_->CloseIfNot(node)) return -1;

  graph_->Expand(node, state);
  ++expanded_;

  if (from_local) ++local_expanded_;

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) return -1;

  state_array.resize(applicable.size());
  packed_array.resize(applicable.size());
  action_array.resize(applicable.size());
  hash_array.resize(applicable.size());
  h_array.resize(applicable.size());
  accepted_array(applicable.size(), graph_->n_landmarks_bytes());

  int index = 0;
  int node_to_keep = -1;

  for (auto o : applicable) {
    auto &child = state_array[index];
    child = state;
    problem_->ApplyEffect(o, child);

    auto &packed = packed_array[index];
    packed.resize(graph_->block_size());
    std::fill(packed.begin(), packed.end(), 0);
    auto &hash = hash_array[index];

    int closed = graph_->GetClosed(o, node, state, child, packed.data(), &hash);

    if (closed != -1) continue;

    uint8_t *accepted = accepted_array[index].data();
    int h = lmcount_->Evaluate(child, graph_->Landmark(node), accepted);

    if (h == -1) continue;

    if (h < local_h_min_)
      node_to_keep = index;

    action_array[index] = o;
    h_values[index] = h;

    index++;
  }

  if (index == 0) return -1;

  for (int i=0; i<index; ++i) {
    ++n_sent_or_generated_;

    auto &child = state_array[i];
    int to_rank = -1;

    if (i != node_to_keep) {
      uint32_t hash = d_hash_->operator()(child);
      to_rank = hash % static_cast<uint32_t>(world_size_);
    }

    int h = h_array[i];
    int a = action_array[i];
    uint32_t c_hash = hash_array[i];
    uint32_t *packed = packed_array[i].data();
    uint8_t *accepted = accepted_array[i].data();

    if (to_rank != -1 && to_rank != rank_) {
      BufferNode(to_rank, h, a, node, c_hash, packed, accepted);
      ++n_sent_;
    }

    if (i == node_to_keep || to_rank == rank_) {
      int child_node = graph_->GenerateNode(a, node, c_hash, packed, rank_);
      graph_->SetLandmark(child_node, accepted);
      ++generated_;

      Push(h, child_node, i == node_to_keep);
    }
  }

  SendNodes(kNodeTag);

  return -1;
}

void HetroHDGBFSLG::Push(int h, int node, bool is_local) {
  thread_local vector<int> values(1);

  values[0] = h;

  if (is_local || h == 0)
    local_node_ = node;
  else
    open_list_->Push(values, node, false);

  graph_->SetH(node, values[0]);

  if (is_local && h < local_h_min_)
    local_h_min_ = h;

  if (h_min_ == -1 || h < h_min_) {
    h_min_ = h;

    if (rank_ == initial_rank_) {
      std::cout << "New best heuristic value: " << h_min_ << std::endl;
      std::cout << "[" << expanded_ << " expanded]" << std::endl;
    }
  }
}

void HetroHDGBFSLG::SendNodes(int tag) {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || outgoing_buffers_[i].empty()) continue;

    unsigned char *d = outgoing_buffers_[i].data();
    MPI_Bsend(d, outgoing_buffers_[i].size(), MPI_BYTE, i, tag, MPI_COMM_WORLD);
    ClearOutgoingBuffer(i);
  }
}

void HetroHDGBFSLG::ReceiveNodes(int tag) {
  int has_received = 0;
  size_t unit_size = node_size() + values.size() * sizeof(int);
  MPI_Status status;
  MPI_Iprobe(MPI_ANY_SOURCE, tag, MPI_COMM_WORLD, &has_received, &status);

  if (has_received) ++n_received_;

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    ResizeIncomingBuffer(d_size);
    MPI_Recv(IncomingBuffer(), d_size, MPI_BYTE, source, tag, MPI_COMM_WORLD,
             MPI_STATUS_IGNORE);

    auto buffer = IncomingBuffer();
    size_t n_nodes = d_size / unit_size;

    for (size_t i=0; i<n_nodes; ++i) {
      int h;
      memcpy(&h, buffer, sizeof(int));
      buffer += sizeof(int);
      int node = graph_->GenerateNodeIfNotClosedFromBytes(buffer);
      buffer += node_size();

      if (node != -1) {
        ++generated_;
        Push(h, node, false);

        if (tag == kNodeWithSequenceTag) {
          int n_steps;
          memcpy(&n_steps, buffer, sizeof(int));
          buffer += sizeof(int);
          sequences_[node] = vector<int>(n_steps);
          memcpy(sequences_[node].data(), buffer, n_steps * sizeof(int));
          buffer += n_steps * sizeof(int);
        }
      }
    }

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, tag, MPI_COMM_WORLD, &has_received, &status);
  }
}

void HetroHDGBFSLG::SendTermination() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_) continue;
    MPI_Bsend(NULL, 0, MPI_BYTE, i,kTerminationTag, MPI_COMM_WORLD);
  }
}

bool HetroHDGBFSLG::ReceiveTermination() {
  int has_received = 0;
  MPI_Iprobe(MPI_ANY_SOURCE, kTerminationTag, MPI_COMM_WORLD, &has_received,
             MPI_STATUS_IGNORE);

  return has_received == 1;
}

vector<int> HetroHDGBFSLG::ExtractPath(int node) {
  vector<int> rec_buffer(world_size_);
  MPI_Gather(
      &node, 1, MPI_INT, rec_buffer.data(), 1, MPI_INT, 0, MPI_COMM_WORLD);
  int goal_process = -1;

  if (rank_ == 0) {
    for (int i=0; i<world_size_; ++i) {
      if (rec_buffer[i] != -1) {
        goal_process = i;
        break;
      }
    }
  }

  MPI_Bcast(&goal_process, 1, MPI_INT, 0, MPI_COMM_WORLD);

  if (goal_process == -1) return vector<int>{-1};

  if (goal_process == rank_) {
    //std::cout << "rank " << rank_ << " has goal node" << std::endl;

    int p[2];
    p[0] = graph_->Parent(node);
    p[1] = graph_->Action(node);
    int p_rank = graph_->ParentRank(node);

    //std::cout << "parent rank " << p_rank << std::endl;
    MPI_Bsend(p, 2, MPI_INT, p_rank, kPlanTag, MPI_COMM_WORLD);
  }

  MPI_Barrier(MPI_COMM_WORLD);
  MPI_Status status;

  while (true) {
    int has_received = 0;
    MPI_Iprobe(
        MPI_ANY_SOURCE, kPlanTag, MPI_COMM_WORLD, &has_received, &status);
    int size;

    if (has_received) {
      //std::cout <<  "rank=" << rank_ << std::endl;
      MPI_Get_count(&status, MPI_INT, &size);
      //std::cout <<  "size=" << size << std::endl;
      int source = status.MPI_SOURCE;
      vector<int> plan(size + 1);
      MPI_Recv(plan.data() + 1, size, MPI_INT, source, kPlanTag, MPI_COMM_WORLD,
               MPI_STATUS_IGNORE);
      int current_node = plan[1];
      //std::cout << "node=" << current_node << std::endl;
      int parent = graph_->Parent(current_node);
      //std::cout << "parent " << parent << std::endl;

      if (parent == -1) {
        for (int i=0; i<world_size_; ++i) {
          if (i == rank_) continue;
          MPI_Bsend(NULL, 0, MPI_BYTE, i, kPlanTerminationTag, MPI_COMM_WORLD);
        }

        plan.erase(plan.begin(), plan.begin() + 2);

        return plan;
      }

      plan[0] = parent;
      plan[1] = graph_->Action(current_node);
      int parent_rank = graph_->ParentRank(current_node);
      //std::cout << "parent rank: " << parent_rank << std::endl;

      MPI_Bsend(plan.data(), size + 1, MPI_INT, parent_rank, kPlanTag,
                MPI_COMM_WORLD);
      //std::cout << std::endl;
    }

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kPlanTerminationTag, MPI_COMM_WORLD,
               &has_received, &status);

    if (has_received) {
      int source = status.MPI_SOURCE;
      MPI_Recv(NULL, 0, MPI_BYTE, source, kPlanTerminationTag, MPI_COMM_WORLD,
               MPI_STATUS_IGNORE);

      return vector<int>(0);
    }
  }

  return vector<int>(0);
}

void HetroHDGBFSLG::Flush(int tag) {
  MPI_Status status;
  int has_received = 0;

  MPI_Iprobe(MPI_ANY_SOURCE, tag, MPI_COMM_WORLD, &has_received, &status);

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    ResizeIncomingBuffer(d_size);
    MPI_Recv(IncomingBuffer(), d_size, MPI_BYTE, source, tag, MPI_COMM_WORLD,
             MPI_STATUS_IGNORE);

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, tag, MPI_COMM_WORLD, &has_received, &status);
  }
}

void HetroHDGBFSLG::DumpStatistics() const {
  int expanded = 0;
  int generated = 0;
  int expanded_local = 0;
  int n_sent = 0;
  int n_sent_or_generated = 0;
  int n_received = 0;

  MPI_Allreduce(&expanded_, &expanded, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&generated_, &generated, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(
      &expanded_local_, &expanded_local, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&n_sent_, &n_sent, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&n_sent_or_generated_, &n_sent_or_generated, 1, MPI_INT,
                MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(
      &n_received_, &n_received, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);

  if (rank_ == initial_rank_) {
    std::cout << "Expanded " << expanded << " state(s)" << std::endl;
    std::cout << "Generated " << generated << " state(s)" << std::endl;
    std::cout << "Expanded from local " << expanded_local
              << " state(s)" << std::endl;

    double local_ratio = static_cast<double>(expanded_local)
      / static_cast<double>(expanded);
    std::cout << "Expand from local ratio " << local_ratio << std::endl;

    double delay = 1.0;

    if (n_received_ > 0)
      delay = static_cast<double>(expanded_) / static_cast<double>(n_received_);

    std::cout << "Delay " << delay << std::endl;

    double co = static_cast<double>(n_sent)
      / static_cast<double>(n_sent_or_generated);

    std::cout << "CO " << co << std::endl;
  }

  graph_->Dump();
}

} // namespace pplanner
