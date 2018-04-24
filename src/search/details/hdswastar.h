#include "../hdswastar.h"

#include <cassert>

#include <limits>
#include <iostream>

#include <mpi.h>

#include "hash/zobrist_hash.h"
#include "domain/state.h"

using std::vector;

namespace rwls {

constexpr int kNodeTag = 0;
constexpr int kTerminationTag = 1;
constexpr int kPlanTag = 2;
constexpr int kPlanTerminationTag = 3;

template<class H>
vector<int> HDSWAstar<H>::operator()() {
  MPI_Comm_size(MPI_COMM_WORLD, &world_size_);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank_);

  outgo_buffer_.resize(world_size_);
  unsigned int buffer_size = (node_size_ + MPI_BSEND_OVERHEAD)
                             * world_size_ * 5000;
  unsigned char *mpi_buffer = new unsigned char[buffer_size];
  MPI_Buffer_attach((void*)mpi_buffer, buffer_size);

  MPI_Barrier(MPI_COMM_WORLD);

  int goal = Search();

  MPI_Barrier(MPI_COMM_WORLD);

  auto result = ExtractPath(goal);

  Flush();

  int detach_size;
  MPI_Buffer_detach(&mpi_buffer, &detach_size);
  delete[] mpi_buffer;

  return result;
}

template<class H>
void HDSWAstar<H>::SendTermination() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_) continue;
    MPI_Bsend(NULL, 0, MPI_BYTE, i,kTerminationTag, MPI_COMM_WORLD);
  }
}

template<class H>
bool HDSWAstar<H>::ReceiveTermination() {
  int has_received = 0;
  MPI_Iprobe(MPI_ANY_SOURCE, kTerminationTag, MPI_COMM_WORLD, &has_received,
             MPI_STATUS_IGNORE);

  return has_received == 1;
}

template<class H>
void HDSWAstar<H>::SendNodes() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || outgo_buffer_[i].empty()) continue;
    unsigned char *d = outgo_buffer_[i].data();
    MPI_Bsend(d, outgo_buffer_[i].size(), MPI_BYTE, i, kNodeTag,
              MPI_COMM_WORLD);
    outgo_buffer_[i].clear();
  }
}

template<class H>
void HDSWAstar<H>::ReceiveNodes() {
  MPI_Status status;
  int has_received = 0;

  MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received, &status);

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    income_buffer_.resize(d_size);
    MPI_Recv(income_buffer_.data(), d_size, MPI_BYTE, source, kNodeTag,
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    int n_nodes = d_size / node_size_;

    for (int i=0; i<n_nodes; ++i)
      BytesToNode(income_buffer_.data() + i * node_size_);

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received,
               &status);
  }
}

template<class H>
void HDSWAstar<H>::BytesToNode(const unsigned char *d) {
  int info[4];
  // info[0] = operator
  // info[1] = parent
  // info[2] = parent rank
  // info[3] = step_size
  memcpy(info, d, 4 * sizeof(int));
  memcpy(tmp_packed_.data(), d + 4 * sizeof(int),
         block_size_ * sizeof(uint32_t));
  if (closed_.Contain(vec_, tmp_packed_.data())) return;
  int node = vec_.GenerateNode(info[0], info[1], tmp_packed_.data());
  vec_.InsertToAnother(info[3], step_);
  ++generated;

  int capacity = vec_.GetCapacity();
  if (parent_rank_.capacity() < capacity) parent_rank_.reserve(capacity);
  parent_rank_.resize(parent_rank_.size() + 1);
  parent_rank_[node] = info[2];

  packer_.Unpack(tmp_packed_.data(), tmp_child_);
  int h = heuristic_(tmp_child_, domain_);
  ++evaluated;

  if (h == std::numeric_limits<int>::max()) {
    ++deadend;
    return;
  }

  int f = weight_ * h + info[3];

  open_.Push(std::make_pair(f, h), node);
}

template<class H>
void HDSWAstar<H>::NodeToBytes(int to_rank, int a, int parent, int step,
                               const uint32_t *packed) {
  int info[4];
  // info[0] = operator
  // info[1] = parent
  // info[2] = parent rank
  // info[3] = parent rank
  info[0] = a;
  info[1] = parent;
  info[2] = rank_;
  info[3] = step;
  size_t size = outgo_buffer_[to_rank].size();
  outgo_buffer_[to_rank].resize(size + node_size_);
  memcpy(outgo_buffer_[to_rank].data() + size, info, 4 * sizeof(int));
  memcpy(outgo_buffer_[to_rank].data() + size + 4 * sizeof(int),
         packed, block_size_ * sizeof(uint32_t));
}

template<class H>
int HDSWAstar<H>::Search() {
  heuristic_.Initialize(domain_);

  int best_seen = std::numeric_limits<int>::max();
  std::vector<int> applicable;

  tmp_packed_.resize(block_size_);
  packer_.Pack(domain_.initial, tmp_packed_.data());
  tmp_state_ = domain_.initial;
  tmp_child_ = domain_.initial;

  ZobristHash z_hash(domain_);
  size_t hash = z_hash(domain_.initial) % static_cast<size_t>(world_size_);
  int initial_rank = static_cast<int>(hash);

  if (rank_ == initial_rank) {
    std::cout << "rank " << rank_ << " has initial node" << std::endl;

    int current_node = vec_.GenerateNode(-1, -1, tmp_packed_.data());
    vec_.InsertToAnother(0, step_);
    parent_rank_.resize(1);
    parent_rank_[0] = -1;
    ++generated;
    int h = heuristic_(domain_.initial, domain_);
    ++evaluated;
    open_.Push(std::make_pair(h, 0), current_node);
  }

  while (!ReceiveTermination()) {
    ReceiveNodes();
    if (open_.IsEmpty()) continue;
    int current_node = open_.Pop();

    uint32_t *current_packed = vec_.GetState(current_node);
    if (closed_.Contain(vec_, current_packed)) continue;
    closed_.Insert(vec_, current_node);

    packer_.Unpack(current_packed, tmp_state_);

    if (GoalCheck(domain_.goal, tmp_state_)) {
      SendTermination();
      return current_node;
    }

    FindFromTable(table_, domain_, tmp_state_, applicable);

    if (applicable.empty()) {
      ++deadend;
      continue;
    } else {
      ++expanded;
    }

    int h_min = std::numeric_limits<int>::max();
    int child_step = step_[current_node] + 1;

    for (auto o : applicable) {
      tmp_child_ = tmp_state_;
      ApplyEffect(domain_.effects[o], tmp_child_);

      packer_.Pack(tmp_child_, tmp_packed_.data());
      if (closed_.Contain(vec_, tmp_packed_.data())) continue;

      size_t hash = z_hash(tmp_child_) % static_cast<size_t>(world_size_);
      int to_rank = static_cast<int>(hash);

      if (to_rank == rank_) {
        int child_node = vec_.GenerateNode(o, current_node, tmp_packed_.data());
        vec_.InsertToAnother(child_step, step_);
        ++generated;

        int capacity = vec_.GetCapacity();
        if (parent_rank_.capacity() < capacity) parent_rank_.reserve(capacity);
        parent_rank_.resize(parent_rank_.size() + 1);
        parent_rank_[child_node] = rank_;

        int h = heuristic_(tmp_child_, domain_);
        ++evaluated;

        if (h == std::numeric_limits<int>::max()) {
          ++deadend;
          continue;
        }

        int f = weight_ * h + child_step;

        if (h < h_min) h_min = h;
        open_.Push(std::make_pair(f, h), child_node);
      } else {
        NodeToBytes(to_rank, o, current_node, child_step, tmp_packed_.data());
      }
    }

    SendNodes();

    if (h_min < best_seen && rank_ == initial_rank) {
      best_seen = h_min;
      std::cout << "New best heuristic value: " << best_seen
                << std::endl;
      std::cout << "[" << evaluated << " evaluated, "
                << expanded << " expanded]" << std::endl;
    }
  }

  return -1;
}

template<class H>
vector<int> HDSWAstar<H>::ExtractPath(int node) {
  int sum_expanded = 0;
  int sum_evaluated = 0;
  int sum_generated = 0;
  int sum_deadend = 0;
  MPI_Allreduce(&expanded, &sum_expanded, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&evaluated, &sum_evaluated, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&generated, &sum_generated, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&deadend, &sum_deadend, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  expanded = sum_expanded;
  evaluated = sum_evaluated;
  generated = sum_generated;
  deadend = sum_deadend;

  int *rec_buffer = new int[world_size_];
  MPI_Gather(&node, 1, MPI_INT, rec_buffer, 1, MPI_INT, 0, MPI_COMM_WORLD);
  int goal_process = 0;

  if (rank_ == 0) {
    for (int i=0; i<world_size_; ++i) {
      if (rec_buffer[i] != -1) {
        goal_process = i;
        break;
      }
    }
  }

  delete[] rec_buffer;

  MPI_Bcast(&goal_process, 1, MPI_INT, 0, MPI_COMM_WORLD);

  if (goal_process == rank_) {
  std::cout << "rank " << rank_ << " has goal node" << std::endl;

    int p[2];
    p[0] = vec_.GetParent(node);
    p[1] = vec_.GetAction(node);
    int p_rank = parent_rank_[node];

    MPI_Bsend(p, 2, MPI_INT, p_rank, kPlanTag, MPI_COMM_WORLD);

    //std::cout << "parent rank size: " << parent_rank_.size() << std::endl;
    //std::cout << "vec size: " << vec_.GetFront() << std::endl;
  }

  MPI_Barrier(MPI_COMM_WORLD);
  MPI_Status status;

  while (true) {
    int has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kPlanTag, MPI_COMM_WORLD, &has_received, &status);
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
      int parent = vec_.GetParent(current_node);
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
      plan[1] = vec_.GetAction(current_node);
      int p_rank = parent_rank_[current_node];
      //std::cout << "parent rank: " << p_rank << std::endl;
      MPI_Bsend(plan.data(), size + 1, MPI_INT, p_rank, kPlanTag,
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

template<class H>
void HDSWAstar<H>::Flush() {
  MPI_Status status;
  int has_received = 0;

  MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received, &status);

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    income_buffer_.resize(d_size);
    MPI_Recv(income_buffer_.data(), d_size, MPI_BYTE, source, kNodeTag,
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received,
               &status);
  }
}

} // namespace rwls
