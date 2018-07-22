#include "search/hdgbfs.h"

#include <algorithm>
#include <iostream>
#include <unordered_set>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"
#include "search_graph_factory.h"

namespace pplanner {

using std::make_shared;
using std::unordered_set;
using std::vector;

void HDGBFS::Init(const boost::property_tree::ptree &pt) {
  MPI_Comm_rank(MPI_COMM_WORLD, &rank_);

  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  vector<boost::property_tree::ptree> evaluator_names;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    evaluator_names.push_back(child.second);
    ++n_evaluators_;
  }

  bool use_landmark = false;
  if (auto opt = pt.get_optional<int>("landmark")) use_landmark = true;

  bool dump_nodes = false;
  if (auto opt = pt.get_optional<int>("dump_nodes")) dump_nodes = true;

  graph_ = DistributedSearchGraphFactory(problem_, closed_exponent,
                                         n_evaluators_, rank_, use_landmark,
                                         dump_nodes);


  for (auto e : evaluator_names)
    evaluators_.push_back(EvaluatorFactory(problem_, graph_, e));

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same")
        preferring_ = evaluators_[0];
      else
        preferring_ = EvaluatorFactory(problem_, graph_, preferring.get());
    }
  }

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option, evaluators_);

  if (auto ram = pt.get_optional<size_t>("ram"))
    graph_->ReserveByRAMSize(ram.get());
  else
    graph_->ReserveByRAMSize(5000000000);

  MPI_Comm_size(MPI_COMM_WORLD, &world_size_);

  unsigned int buffer_size = 1000000000;
  mpi_buffer_ = new unsigned char[buffer_size];
  MPI_Buffer_attach((void*)mpi_buffer_, buffer_size);

  outgoing_buffers_.resize(world_size_);
}

int HDGBFS::Search() {
  auto state = InitialEvaluate();

  while (!ReceiveTermination()) {
    ReceiveNodes();
    if (NoNode()) continue;

    int node = Pop();
    int goal = Expand(node, state);

    if (goal != -1) {
      SendTermination();

      return goal;
    }
  }

  return -1;
}

vector<int> HDGBFS::InitialEvaluate(bool eager_dd) {
  auto state = problem_->initial();
  uint32_t world_size = static_cast<uint32_t>(world_size_);
  int initial_rank_ = z_hash_->operator()(state) % world_size;

  if (rank_ == initial_rank_) {
    int node = -1;

    if (eager_dd)
      node = graph_->GenerateAndCloseNode(-1, -1, state, -1);
    else
      node = graph_->GenerateNode(-1, -1, state, -1);

    IncrementGenerated();
    int h = open_list_->EvaluateAndPush(state, node, true);
    set_best_h(h);
    std::cout << "Initial heuristic value: " << best_h() << std::endl;
    ++evaluated_;
  }

  return state;
}

int HDGBFS::Expand(int node, vector<int> &state, bool eager_dd) {
  static vector<int> values;
  static vector<int> child;
  static vector<int> applicable;
  static unordered_set<int> preferred;

  if (!eager_dd && !graph_->CloseIfNot(node)) return -1;

  ++expanded_;
  graph_->Expand(node, state);

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    IncrementDeadEnds();
    return -1;
  }

  if (use_preferred_)
    preferring_->Evaluate(state, node, applicable, preferred);

  ++n_preferred_evaluated_;
  n_branching_ += applicable.size();

  for (auto o : applicable) {
    child = state;
    problem_->ApplyEffect(o, child);

    bool is_preferred = use_preferred_ && preferred.find(o) != preferred.end();
    if (is_preferred) ++n_preferreds_;

    uint32_t hash = z_hash_->operator()(child);
    int to_rank = hash % static_cast<uint32_t>(world_size_);

    if (to_rank == rank_) {
      int child_node = -1;

      if (eager_dd) {
        child_node = graph_->GenerateAndCloseNode(o, node, state, child, rank_);
      } else {
        child_node = graph_->GenerateNodeIfNotClosed(
            o, node, state, child, rank_);
      }

      if (child_node == -1) continue;
      IncrementGenerated();
      int h = Evaluate(child, child_node, values);
      if (h != -1) Push(values, child_node);
    } else {
      unsigned char *buffer = ExtendOutgoingBuffer(to_rank, node_size());
      graph_->BufferNode(o, node, state, child, buffer);
    }
  }

  SendNodes(kNodeTag);

  return -1;
}

int HDGBFS::Evaluate(const vector<int> &state, int node, vector<int> &values) {
  ++evaluated_;

  values.clear();

  for (auto evaluator : evaluators_) {
    int value = evaluator->Evaluate(state, node);

    if (value == -1) {
      IncrementDeadEnds();

      return value;
    }

    values.push_back(value);
  }

  return values[0];
}


void HDGBFS::Push(std::vector<int> &values, int node) {
  int h = values[0];
  open_list_->Push(values, node, false);

  if (best_h() == -1 || h < best_h()) {
    set_best_h(h);

    if (rank_ == initial_rank_) {
      std::cout << "New best heuristic value: " << best_h() << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, "
                << expanded_ << " expanded]" << std::endl;
    }

    if (use_preferred_) open_list_->Boost();
  }
}

void HDGBFS::SendNodes(int tag) {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || IsOutgoingBufferEmpty(i)) continue;
    const unsigned char *d = OutgoingBuffer(i);
    MPI_Bsend(d, OutgoingBufferSize(i), MPI_BYTE, i, tag, MPI_COMM_WORLD);
    ClearOutgoingBuffer(i);
  }
}

void HDGBFS::ReceiveNodes() {
  int has_received = 0;
  size_t unit_size = node_size();
  MPI_Status status;
  MPI_Iprobe(MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received, &status);

  if (has_received) ++n_received_;

  while (has_received) {
    int d_size = 0;
    int source = status.MPI_SOURCE;
    MPI_Get_count(&status, MPI_BYTE, &d_size);
    ResizeIncomingBuffer(d_size);
    MPI_Recv(IncomingBuffer(), d_size, MPI_BYTE, source, kNodeTag,
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    size_t n_nodes = d_size / unit_size;
    bool no_node = NoNode();

    for (size_t i=0; i<n_nodes; ++i)
      CallbackOnReceiveNode(source, IncomingBuffer() + i * unit_size, no_node);

    has_received = 0;
    MPI_Iprobe(
        MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received, &status);
  }

  CallbackOnReceiveAllNodes();
}

void HDGBFS::CallbackOnReceiveNode(int source, const unsigned char *d,
                                   bool no_node) {
  static vector<int> values;

  int node = graph_->GenerateNodeIfNotClosed(d);

  if (node != -1) {
    IncrementGenerated();
    graph_->State(node, tmp_state_);
    int h = Evaluate(tmp_state_, node, values);

    if (h == -1) {
      IncrementDeadEnds();

      return;
    }

    Push(values, node);
  }
}

void HDGBFS::SendTermination() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_) continue;
    MPI_Bsend(NULL, 0, MPI_BYTE, i,kTerminationTag, MPI_COMM_WORLD);
  }
}

bool HDGBFS::ReceiveTermination() {
  int has_received = 0;
  MPI_Iprobe(MPI_ANY_SOURCE, kTerminationTag, MPI_COMM_WORLD, &has_received,
             MPI_STATUS_IGNORE);

  return has_received == 1;
}

vector<int> HDGBFS::ExtractPath(int node) {
  vector<int> rec_buffer(world_size_);
  MPI_Gather(
      &node, 1, MPI_INT, rec_buffer.data(), 1, MPI_INT, 0, MPI_COMM_WORLD);
  int goal_process = 0;

  if (rank_ == initial_rank_) {
    for (int i=0; i<world_size_; ++i) {
      if (rec_buffer[i] != -1) {
        goal_process = i;
        break;
      }
    }
  }

  MPI_Bcast(&goal_process, 1, MPI_INT, 0, MPI_COMM_WORLD);

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

void HDGBFS::Flush(int tag) {
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

void HDGBFS::DumpStatistics() const {
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;
  int n_preferreds = 0;
  int n_preferred_evaluated = 0;
  int n_branching = 0;
  int n_received = 0;
  MPI_Allreduce(&expanded_, &expanded, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&evaluated_, &evaluated, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&generated_, &generated, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&dead_ends_, &dead_ends, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(
      &n_preferreds_, &n_preferreds, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&n_preferred_evaluated_, &n_preferred_evaluated, 1, MPI_INT,
                MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(
      &n_branching_, &n_branching, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(
      &n_received_, &n_received, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);

  if (rank_ == initial_rank_) {
    std::cout << "Expanded " << expanded << " state(s)" << std::endl;
    std::cout << "Evaluated " << evaluated << " state(s)" << std::endl;
    std::cout << "Generated " << generated << " state(s)" << std::endl;
    std::cout << "Dead ends " << dead_ends << " state(s)" << std::endl;
    std::cout << "Preferred evaluated " << n_preferred_evaluated_ << " state(s)"
              << std::endl;
    std::cout << "Preferred successors " << n_preferreds_ << " state(s)"
              << std::endl;
    double p_p_e = static_cast<double>(n_preferreds)
      / static_cast<double>(n_preferred_evaluated);
    std::cout << "Preferreds per state " << p_p_e << std::endl;
    double b_f = static_cast<double>(n_branching)
      / static_cast<double>(n_preferred_evaluated);
    std::cout << "Average branching factor " << b_f << std::endl;
    double p_p_b = static_cast<double>(n_preferreds)
      / static_cast<double>(n_branching);
    std::cout << "Preferred ratio " << p_p_b  << std::endl;

    double delay = 1.0;

    if (n_received_ > 0)
      delay = static_cast<double>(expanded_) / static_cast<double>(n_received_);

    std::cout << "Delay " << delay << std::endl;
  }

  graph_->Dump();
}

} // namespace pplanner
