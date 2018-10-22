#include "search/ashdgbfs.h"

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
using std::unordered_set;
using std::vector;

void ASHDGBFS::Init(const boost::property_tree::ptree &pt) {
  MPI_Comm_rank(MPI_COMM_WORLD, &rank_);

  if (auto opt = pt.get_optional<int>("max_expansion")) {
    limit_expansion_ = true;
    max_expansion_ = opt.get();
  }

  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  vector<boost::property_tree::ptree> evaluator_names;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    evaluator_names.push_back(child.second);
    ++n_evaluators_;
  }

  bool keep_cost = false;
  if (auto opt = pt.get_optional<int>("keep_cost")) keep_cost = true;

  bool use_landmark = false;
  if (auto opt = pt.get_optional<int>("landmark")) use_landmark = true;

  bool dump_nodes = false;
  if (auto opt = pt.get_optional<int>("dump_nodes")) dump_nodes = true;

  graph_ = DistributedSearchGraphFactory(problem_, closed_exponent,
                                         n_evaluators_, rank_, keep_cost,
                                         use_landmark, dump_nodes);

  std::shared_ptr<Evaluator> friend_evaluator = nullptr;

  for (auto e : evaluator_names) {
    auto evaluator = EvaluatorFactory(problem_, graph_, friend_evaluator,  e);
    evaluators_.push_back(evaluator);
    friend_evaluator = evaluator;
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same") {
        preferring_ = evaluators_[0];
      } else {
        preferring_ = EvaluatorFactory(
            problem_, graph_, nullptr, preferring.get());
      }
    }
  }

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option, evaluators_);

  if (auto ram = pt.get_optional<size_t>("ram"))
    graph_->ReserveByRAMSize(ram.get());
  else
    graph_->ReserveByRAMSize(5000000000);

  if (auto opt = pt.get_optional<int>("sss")) {
    use_sss_ = true;
    sss_aproximater_ = std::unique_ptr<SSSApproximater>(
        new SSSApproximater(problem_));
  }

  if (auto opt = pt.get_optional<int>("send_once"))
    if (opt.get() == 0) send_once_ = false;

  std::string abstraction = "none";

  if (auto opt = pt.get_optional<std::string>("abstraction"))
    abstraction = opt.get();

  z_hash_ = DistributionHashFactory(problem_, 2886379259, abstraction);

  MPI_Comm_size(MPI_COMM_WORLD, &world_size_);

  unsigned int buffer_size = 1000000000;
  mpi_buffer_ = new unsigned char[buffer_size];
  MPI_Buffer_attach((void*)mpi_buffer_, buffer_size);

  open_min_h_in_proc_.resize(world_size_, -1);
  outgoing_buffers_.resize(world_size_,
                           std::vector<unsigned char>(sizeof(int)));

  for (auto &buffer : outgoing_buffers_) {
    int h = -1;
    memcpy(buffer.data(), &h, sizeof(int));
  }
}

int ASHDGBFS::Search() {
  auto state = InitialEvaluate();

  while (!ReceiveTermination()) {
    ReceiveNodes();
    if (NoNode()) continue;

    //std::cout << "rank=" << rank_ << ", h=" << MinimumValues()[0] << std::endl;
    int node = Pop();
    int goal = Expand(node, state);

    if (goal != -1 || (limit_expansion() && expanded() > max_expansion())) {
      SendTermination();

      return goal;
    }

    SendNodes(kNodeTag);
  }

  return -1;
}

vector<int> ASHDGBFS::InitialEvaluate() {
  auto state = problem_->initial();
  uint32_t world_size = static_cast<uint32_t>(world_size_);
  int initial_rank_ = z_hash_->operator()(state) % world_size;

  if (rank_ == initial_rank_) {
    int node = -1;
    node = graph_->GenerateNode(-1, -1, state, -1);
    IncrementGenerated();
    int h = open_list_->EvaluateAndPush(state, node, true);
    open_min_h_in_proc_[rank_] = h;
    graph_->SetH(node, h);
    set_best_h(h);
    std::cout << "Initial heuristic value: " << best_h() << std::endl;
    ++evaluated_;
  }

  return state;
}

int ASHDGBFS::FindEmptyProcess() const {
  for (int i=0; i<world_size_; ++i)
    if (open_min_h_in_proc_[i] == -1) return i;

  return -1;
}

int ASHDGBFS::GlobalMinH() const {
  int min_h = -1;

  for (int i=0; i<world_size_; ++i) {
    int h = open_min_h_in_proc_[i];

    if (h != -1 && (h < min_h || min_h == -1))
      min_h = h;
  }

  return min_h;
}

int ASHDGBFS::Expand(int node, vector<int> &state) {
  static vector<int> values;
  static vector<int> child;
  static vector<int> applicable;
  static unordered_set<int> preferred;
  static vector<bool> sss;

  if (!graph_->CloseIfNot(node)) return -1;

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

  if (use_sss_)
    sss_aproximater_->ApproximateSSS(state, applicable, sss);

  for (auto o : applicable) {
    if (use_sss_ && !sss[o]) continue;

    child = state;
    problem_->ApplyEffect(o, child);

    int child_node = graph_->GenerateNodeIfNotClosed(
        o, node, state, child, rank_);

    if (child_node == -1) continue;

    IncrementGenerated();
    int h = Evaluate(child, child_node, values);

    if (h == -1) {
      IncrementDeadEnds();
      continue;
    }

    ++n_sent_or_generated_;
    int global_h_min = GlobalMinH();
    int to_rank = FindEmptyProcess();

    if (global_h_min != -1 && h >= global_h_min && to_rank == -1) {
      Push(values, child_node);
      continue;
    }

    if (to_rank == -1) {
      uint32_t hash = z_hash_->operator()(child);
      to_rank = hash % static_cast<uint32_t>(world_size_);
    }

    if (to_rank == rank_) {
      Push(values, child_node);
    } else {
      unsigned char *buffer = ExtendOutgoingBuffer(
          to_rank, node_size() + values.size() * sizeof(int));
      memcpy(buffer, values.data(), values.size() * sizeof(int));
      graph_->BufferNode(child_node, buffer + values.size() * sizeof(int));
      UpdateOpenMinH(to_rank, h);
      ++n_sent_;
    }
  }

  return -1;
}

int ASHDGBFS::Evaluate(const vector<int> &state, int node, vector<int> &values) {
  ++evaluated_;

  values.clear();

  for (auto evaluator : evaluators_) {
    int value = evaluator->Evaluate(state, node);

    if (value == -1) {
      IncrementDeadEnds();
      graph_->SetH(node, value);

      return value;
    }

    values.push_back(value);
  }

  graph_->SetH(node, values[0]);

  return values[0];
}


void ASHDGBFS::Push(std::vector<int> &values, int node) {
  int h = values[0];
  open_list_->Push(values, node, false);
  UpdateOpenMinH(rank_, h);

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

void ASHDGBFS::SendNodes(int tag) {
  if (NoNode()) open_min_h_in_proc_[rank_] = -1;

  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || outgoing_buffers_[i].size() == sizeof(int)) continue;

    unsigned char *d = outgoing_buffers_[i].data();
    memcpy(d, open_min_h_in_proc_.data() + rank_, sizeof(int));
    MPI_Bsend(d, outgoing_buffers_[i].size(), MPI_BYTE, i, tag, MPI_COMM_WORLD);
    ClearOutgoingBuffer(i);
  }
}

void ASHDGBFS::ReceiveNodes() {
  static vector<int> values(n_evaluators());

  int has_received = 0;
  size_t unit_size = node_size() + values.size() * sizeof(int);
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

    auto buffer = IncomingBuffer();

    memcpy(open_min_h_in_proc_.data() + source, buffer, sizeof(int));
    buffer += sizeof(int);

    size_t n_nodes = (d_size - sizeof(int)) / unit_size;

    for (size_t i=0; i<n_nodes; ++i) {
      memcpy(values.data(), buffer, values.size() * sizeof(int));
      buffer += values.size() * sizeof(int);
      int node = graph_->GenerateNodeIfNotClosedFromBytes(buffer);

      if (node != -1) {
        IncrementGenerated();
        Push(values, node);
      }

      buffer += node_size();
    }

    has_received = 0;
    MPI_Iprobe(
        MPI_ANY_SOURCE, kNodeTag, MPI_COMM_WORLD, &has_received, &status);
  }
}

void ASHDGBFS::SendTermination() {
  for (int i=0; i<world_size_; ++i) {
    if (i == rank_) continue;
    MPI_Bsend(NULL, 0, MPI_BYTE, i,kTerminationTag, MPI_COMM_WORLD);
  }
}

bool ASHDGBFS::ReceiveTermination() {
  int has_received = 0;
  MPI_Iprobe(MPI_ANY_SOURCE, kTerminationTag, MPI_COMM_WORLD, &has_received,
             MPI_STATUS_IGNORE);

  return has_received == 1;
}

vector<int> ASHDGBFS::ExtractPath(int node) {
  vector<int> rec_buffer(world_size_);
  MPI_Gather(
      &node, 1, MPI_INT, rec_buffer.data(), 1, MPI_INT, 0, MPI_COMM_WORLD);
  int goal_process = -1;

  if (rank_ == initial_rank_) {
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

void ASHDGBFS::Flush(int tag) {
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

void ASHDGBFS::DumpStatistics() const {
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;
  int n_preferreds = 0;
  int n_preferred_evaluated = 0;
  int n_branching = 0;
  int n_sent = 0;
  int n_sent_or_generated = 0;
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
  MPI_Allreduce(&n_sent_, &n_sent, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  MPI_Allreduce(&n_sent_or_generated_, &n_sent_or_generated, 1, MPI_INT,
                MPI_SUM, MPI_COMM_WORLD);
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

    double co = static_cast<double>(n_sent)
      / static_cast<double>(n_sent_or_generated);

    std::cout << "CO " << co << std::endl;
  }

  graph_->Dump();
}

} // namespace pplanner
