#include "../pddsbgbfs.h"

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
constexpr int kDDTag = 4;

template<class H>
vector<int> PDDSBGBFS<H>::operator()() {
  MPI_Comm_size(MPI_COMM_WORLD, &world_size_);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank_);

  outgo_buffer_.resize(world_size_);
  unsigned int buffer_size = (node_size_ + MPI_BSEND_OVERHEAD)
                             * world_size_ * 5000;
  unsigned char *mpi_buffer = new unsigned char[buffer_size];
  MPI_Buffer_attach((void*)mpi_buffer, buffer_size);

  MPI_Barrier(MPI_COMM_WORLD);

  int goal = Init();

  if (goal == -1)
    goal = Search();

  MPI_Barrier(MPI_COMM_WORLD);

  auto result = ExtractPath(goal);

  Flush();

  int detach_size;
  MPI_Buffer_detach(&mpi_buffer, &detach_size);
  delete[] mpi_buffer;

  return result;
}

template<class H>
void PDDSBGBFS<H>::SendTermination() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_) continue;
    MPI_Bsend(NULL, 0, MPI_BYTE, i, kTerminationTag, MPI_COMM_WORLD);
  }
}

template<class H>
bool PDDSBGBFS<H>::ReceiveTermination() {
  int has_received = 0;
  MPI_Iprobe(MPI_ANY_SOURCE, kTerminationTag, MPI_COMM_WORLD, &has_received,
             MPI_STATUS_IGNORE);

  return has_received == 1;
}

template<class H>
void PDDSBGBFS<H>::SendNodes() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || outgo_buffer_[i].empty()) continue;
    unsigned char *d = outgo_buffer_[i].data();
    MPI_Bsend(d, outgo_buffer_[i].size(), MPI_BYTE, i, kNodeTag,
              MPI_COMM_WORLD);
    outgo_buffer_[i].clear();
  }
}

template<class H>
void PDDSBGBFS<H>::ReceiveNodes() {
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

    int n_nodes = d_size / (sizeof(int) + node_size_);

    for (int i=0; i<n_nodes; ++i)
      BytesToNode(income_buffer_.data() + i * (sizeof(int) + node_size_));

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received,
               &status);
  }
}

template<class H>
void PDDSBGBFS<H>::BytesToNode(const unsigned char *d) {
  int info[4];
  // info[0] = h
  // info[1] = operator
  // info[2] = parent
  // info[3] = parent rank
  memcpy(info, d, 4 * sizeof(int));
  memcpy(tmp_packed_.data(), d + 4 * sizeof(int),
         block_size_ * sizeof(uint32_t));
  int node = vec_.GenerateNode(info[1], info[2], tmp_packed_.data());
  ++generated;

  int capacity = vec_.GetCapacity();
  if (parent_rank_.capacity() < capacity) parent_rank_.reserve(capacity);
  parent_rank_.resize(parent_rank_.size() + 1);
  parent_rank_[node] = info[3];
  packer_.Unpack(tmp_packed_.data(), tmp_child_);

  int h = info[0];
  if (h < h_min_) h_min_ = h;

  open_.Push(h, node);
}

template<class H>
void PDDSBGBFS<H>::BytesToNode(int h, const unsigned char *d) {
  int info[3];
  // info[0] = operator
  // info[1] = parent
  // info[2] = parent rank
  memcpy(info, d, 3 * sizeof(int));
  memcpy(tmp_packed_.data(), d + 3 * sizeof(int),
         block_size_ * sizeof(uint32_t));
  int node = vec_.GenerateNode(info[0], info[1], tmp_packed_.data());
  ++generated;

  int capacity = vec_.GetCapacity();
  if (parent_rank_.capacity() < capacity) parent_rank_.reserve(capacity);
  parent_rank_.resize(parent_rank_.size() + 1);
  parent_rank_[node] = info[2];
  packer_.Unpack(tmp_packed_.data(), tmp_child_);

  open_.Push(h, node);
}

template<class H>
void PDDSBGBFS<H>::NodeToBytes(int to_rank, int a, int parent,
                               const uint32_t *packed) {
  int info[3];
  // info[0] = operator
  // info[1] = parent
  // info[2] = parent rank
  info[0] = a;
  info[1] = parent;
  info[2] = rank_;
  size_t size = outgo_buffer_[to_rank].size();
  outgo_buffer_[to_rank].resize(size + node_size_);
  memcpy(outgo_buffer_[to_rank].data() + size, info, 3 * sizeof(int));
  memcpy(outgo_buffer_[to_rank].data() + size + 3 * sizeof(int),
         packed, block_size_ * sizeof(uint32_t));
}

template<class H>
void PDDSBGBFS<H>::SendDD() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || outgo_buffer_[i].empty()) continue;
    unsigned char *d = outgo_buffer_[i].data();
    MPI_Bsend(d, outgo_buffer_[i].size(), MPI_BYTE, i, kDDTag,
              MPI_COMM_WORLD);
    outgo_buffer_[i].clear();
  }
}

template<class H>
void PDDSBGBFS<H>::ReceiveDD() {
  MPI_Status status;
  int has_received = 0;

  MPI_Iprobe(MPI_ANY_SOURCE, kDDTag, MPI_COMM_WORLD, &has_received, &status);

  if (!has_received) return;

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    income_buffer_.resize(d_size);
    MPI_Recv(income_buffer_.data(), d_size, MPI_BYTE, source, kDDTag,
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    int n_nodes = d_size / node_size_;

    bool steal = open_.IsEmpty();

    for (int i=0; i<n_nodes; ++i) {
      unsigned char *d = income_buffer_.data() + i * node_size_;
      int info[3];
      memcpy(info, d, 3 * sizeof(int));
      memcpy(tmp_packed_.data(), d + 3 * sizeof(int),
             block_size_ * sizeof(uint32_t));

      if (BytesDD(info, tmp_packed_.data())) continue;

      packer_.Unpack(tmp_packed_.data(), tmp_child_);
      int h = heuristic_(tmp_child_, domain_);
      ++evaluated;

      if (h == std::numeric_limits<int>::max()) {
        ++deadend;
        continue;
      }

      if (h < h_min_) {
        h_min_ = h;
        BytesToNode(h, d);
        //std::cout << "steal by h" << std::endl;
      } else if (steal) {
        BytesToNode(h, d);
        //std::cout << "steal" << std::endl;
      } else {
        size_t size = outgo_buffer_[source].size();
        outgo_buffer_[source].resize(size + sizeof(int) + node_size_);
        memcpy(outgo_buffer_[source].data() + size, &h, sizeof(int));
        memcpy(outgo_buffer_[source].data() + size + sizeof(int), d,
               3 * sizeof(int) + block_size_ * sizeof(uint32_t));
      }
    }

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kDDTag, MPI_COMM_WORLD, &has_received, &status);
  }

  SendNodes();
}

template<class H>
bool PDDSBGBFS<H>::BytesDD(int *info, const uint32_t *state) {
  bool result = closed_.Contain(vec_, state);

  if (!result) {
    // info[0] = operator
    // info[1] = parent
    // info[2] = parent rank
    int node = vec_.GenerateNode(info[0], info[1], state);

    int capacity = vec_.GetCapacity();
    if (parent_rank_.capacity() < capacity) parent_rank_.reserve(capacity);
    parent_rank_.resize(parent_rank_.size() + 1);
    parent_rank_[node] = info[2];

    closed_.Insert(vec_, node);
  }

  return result;
}

template<class H>
int PDDSBGBFS<H>::Init() {
  heuristic_.Initialize(domain_);

  int best_seen = std::numeric_limits<int>::max();
  std::vector<int> applicable;

  tmp_packed_.resize(block_size_);
  packer_.Pack(domain_.initial, tmp_packed_.data());
  tmp_state_ = domain_.initial;
  tmp_child_ = domain_.initial;

  int goal_node = -1;

  if (rank_ == 0) {
    int current_node = vec_.GenerateNode(-1, -1, tmp_packed_.data());
    parent_rank_.resize(1);
    parent_rank_[0] = -1;
    ++generated;
    int h = heuristic_(domain_.initial, domain_);
    ++evaluated;
    h_min_ = h;
    open_.Push(h, current_node);

    while (open_.Size() < world_size_) {
      if (open_.IsEmpty()) continue;
      int current_node = open_.Pop();

      uint32_t *current_packed = vec_.GetState(current_node);
      packer_.Unpack(current_packed, tmp_state_);

      if (GoalCheck(domain_.goal, tmp_state_)) {
        goal_node = current_node;
        SendTermination();
        break;
      }

      FindFromTable(table_, domain_, tmp_state_, applicable);

      if (applicable.empty()) {
        ++deadend;
        continue;
      } else {
        ++expanded;
      }

      for (auto o : applicable) {
        tmp_child_ = tmp_state_;
        ApplyEffect(domain_.effects[o], tmp_child_);

        packer_.Pack(tmp_child_, tmp_packed_.data());
        if (closed_.Contain(vec_, tmp_packed_.data())) continue;

        int child_node = vec_.GenerateNode(o, current_node, tmp_packed_.data());
        closed_.Insert(vec_, child_node);
        ++generated;

        int capacity = vec_.GetCapacity();
        if (parent_rank_.capacity() < capacity) parent_rank_.reserve(capacity);
        parent_rank_.resize(parent_rank_.size() + 1);
        parent_rank_[child_node] = rank_;

        int h = heuristic_(tmp_child_, domain_);
        ++evaluated;
        if (h < h_min_) h_min_ = h;

        if (h == std::numeric_limits<int>::max()) {
          ++deadend;
          continue;
        }

        open_.Push(h, child_node);
      }
    }

    if (goal_node == -1) {
      int i = 1;
      bool distributed = false;

      while (!distributed && !open_.IsEmpty()) {
        int current_node = open_.Pop();

        uint32_t *current_packed = vec_.GetState(current_node);
        packer_.Unpack(current_packed, tmp_state_);

        if (GoalCheck(domain_.goal, tmp_state_)) {
          goal_node = current_node;
          SendTermination();
          break;
        }

        FindFromTable(table_, domain_, tmp_state_, applicable);

        if (applicable.empty()) {
          ++deadend;
          continue;
        } else {
          ++expanded;
        }

        for (auto o : applicable) {
          tmp_child_ = tmp_state_;
          ApplyEffect(domain_.effects[o], tmp_child_);

          packer_.Pack(tmp_child_, tmp_packed_.data());
          if (closed_.Contain(vec_, tmp_packed_.data())) continue;

          NodeToBytes(i, o, current_node, tmp_packed_.data());

          ++i;

          if (i == world_size_) {
            distributed = true;
            i = 1;
          }
        }
      }

      if (goal_node == -1) SendDD();
    }
  }

  return goal_node;
}

template<class H>
int PDDSBGBFS<H>::Search() {
  ZobristHash z_hash(domain_);

  int best_seen = std::numeric_limits<int>::max();
  std::vector<int> applicable;

  while (!ReceiveTermination()) {
    ReceiveDD();
    ReceiveNodes();
    if (open_.IsEmpty()) continue;
    int current_node = open_.Pop();

    uint32_t *current_packed = vec_.GetState(current_node);

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

    for (auto o : applicable) {
      tmp_child_ = tmp_state_;
      ApplyEffect(domain_.effects[o], tmp_child_);
      size_t hash = z_hash(tmp_child_) % static_cast<size_t>(world_size_);
      int to_rank = static_cast<int>(hash);
      packer_.Pack(tmp_child_, tmp_packed_.data());

      if (to_rank == rank_) {
        if (closed_.Contain(vec_, tmp_packed_.data())) continue;

        int child_node = vec_.GenerateNode(o, current_node, tmp_packed_.data());
        closed_.Insert(vec_, child_node);
        ++generated;

        int capacity = vec_.GetCapacity();
        if (parent_rank_.capacity() < capacity) parent_rank_.reserve(capacity);
        parent_rank_.resize(parent_rank_.size() + 1);
        parent_rank_[child_node] = rank_;

        int h = heuristic_(tmp_child_, domain_);
        ++evaluated;
        if (h < h_min_) h_min_ = h;

        if (h == std::numeric_limits<int>::max()) {
          ++deadend;
          continue;
        }

        if (h < h_min) h_min = h;
        open_.Push(h, child_node);
      } else {
        NodeToBytes(to_rank, o, current_node, tmp_packed_.data());
      }
    }

    SendDD();

    if (h_min < best_seen && rank_ == 0) {
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
vector<int> PDDSBGBFS<H>::ExtractPath(int node) {
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
void PDDSBGBFS<H>::Flush() {
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

  MPI_Iprobe(MPI_ANY_SOURCE, kDDTag, MPI_COMM_WORLD, &has_received, &status);

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    income_buffer_.resize(d_size);
    MPI_Recv(income_buffer_.data(), d_size, MPI_BYTE, source, kDDTag,
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    has_received = 0;
    MPI_Iprobe(MPI_ANY_SOURCE, kDDTag, MPI_COMM_WORLD, &has_received, &status);
  }
}

} // namespace rwls
