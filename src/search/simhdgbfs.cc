#include "search/simhdgbfs.h"

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

void SIMHDGBFS::Init(const boost::property_tree::ptree &pt) {
  world_size_ = pt.get<int>("n");

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

  size_t ram = 5000000000 / world_size_;

  if (auto opt = pt.get_optional<size_t>("ram"))
    ram = opt.get() / world_size_;

  auto open_list_option = pt.get_child("open_list");

  bool dump_nodes = false;
  if (auto opt = pt.get_optional<int>("dump_nodes")) dump_nodes = true;

  graphs_.resize(world_size_, nullptr);
  evaluators_.resize(world_size_);

  for (int i=0; i<world_size_; ++i) {
    rank_ = i;
    graphs_[rank_] = DistributedSearchGraphFactory(
        problem_, closed_exponent, n_evaluators_, rank_, keep_cost,
        use_landmark, dump_nodes);

    std::shared_ptr<Evaluator> friend_evaluator = nullptr;

    for (auto e : evaluator_names) {
      auto evaluator = EvaluatorFactory(
          problem_, graphs_[rank_], friend_evaluator, e);
      evaluators_[rank_].push_back(evaluator);
      friend_evaluator = evaluator;
    }

    open_lists_.push_back(
        OpenListFactory(open_list_option, evaluators_[rank_]));

    graphs_[rank_]->ReserveByRAMSize(ram);
  }

  std::string abstraction = "none";

  if (auto opt = pt.get_optional<std::string>("abstraction"))
    abstraction = opt.get();

  z_hash_ = DistributionHashFactory(problem_, 2886379259, abstraction);

  outgoing_buffers_.resize(world_size_,
                           vector<vector<unsigned char> >(world_size_));
}

int SIMHDGBFS::Search() {
  auto state = InitialEvaluate();

  while (true) {
    for (int i=0; i<world_size_; ++i) {
      rank_ = i;
      CommunicateNodes();
    }

    for (int i=0; i<world_size_; ++i) {
      rank_ = i;
      int node = -1;

      while (node == -1 && !open_lists_[rank_]->IsEmpty()) {
        node = open_lists_[rank_]->Pop();

        if (graphs_[rank_]->GetClosed(node) == -1)
          graphs_[rank_]->Close(node);
        else
          node = -1;
      }

      if (node == -1) continue;

      int goal = Expand(node, state);

      if (goal != -1 || (limit_expansion() && expanded() > max_expansion()))
        return goal;
    }
  }

  return -1;
}

vector<int> SIMHDGBFS::InitialEvaluate() {
  auto state = problem_->initial();
  uint32_t world_size = static_cast<uint32_t>(world_size_);
  rank_ = z_hash_->operator()(state) % world_size;

  int node = graphs_[rank_]->GenerateNode(-1, -1, state, -1);

  IncrementGenerated();
  int h = open_lists_[rank_]->EvaluateAndPush(state, node, true);
  graphs_[rank_]->SetH(node, h);
  set_best_h(h);
  std::cout << "Initial heuristic value: " << best_h() << std::endl;
  ++evaluated_;

  return state;
}

int SIMHDGBFS::Expand(int node, vector<int> &state) {
  static vector<int> values;
  static vector<int> child;
  static vector<int> applicable;
  static unordered_set<int> preferred;
  static vector<bool> sss;

  ++expanded_;
  graphs_[rank_]->Expand(node, state);

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    IncrementDeadEnds();
    return -1;
  }

  n_branching_ += applicable.size();

  for (auto o : applicable) {
    child = state;
    problem_->ApplyEffect(o, child);

    uint32_t hash = z_hash_->operator()(child);
    int to_rank = hash % static_cast<uint32_t>(world_size_);

    ++n_sent_or_generated_;

    if (to_rank == rank_) {
      int child_node = -1;

      child_node = graphs_[rank_]->GenerateNodeIfNotClosed(
          o, node, state, child, rank_);

      if (child_node == -1) continue;
      IncrementGenerated();
      int h = Evaluate(child, child_node, values);
      if (h != -1) Push(values, child_node);
    } else {
      unsigned char *buffer = ExtendOutgoingBuffer(to_rank, node_size());
      graphs_[rank_]->BufferNode(o, node, state, child, buffer);
      ++n_sent_;
    }
  }

  return -1;
}

int SIMHDGBFS::Evaluate(const vector<int> &state, int node, vector<int> &values) {
  ++evaluated_;

  values.clear();

  for (auto evaluator : evaluators_[rank_]) {
    int value = evaluator->Evaluate(state, node);

    if (value == -1) {
      IncrementDeadEnds();
      graphs_[rank_]->SetH(node, value);

      return value;
    }

    values.push_back(value);
  }

  graphs_[rank_]->SetH(node, values[0]);

  return values[0];
}


void SIMHDGBFS::Push(std::vector<int> &values, int node) {
  int h = values[0];
  open_lists_[rank_]->Push(values, node, false);

  if (best_h() == -1 || h < best_h()) {
    set_best_h(h);

   std::cout << "New best heuristic value: " << best_h() << std::endl;
   std::cout << "[" << evaluated_ << " evaluated, "
             << expanded_ << " expanded]" << std::endl;
  }
}

void SIMHDGBFS::CommunicateNodes() {
  size_t unit_size = node_size();

  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || outgoing_buffers_[i][rank_].empty()) continue;

    size_t d_size = outgoing_buffers_[i][rank_].size();
    ResizeIncomingBuffer(d_size);
    memcpy(IncomingBuffer(), outgoing_buffers_[i][rank_].data(), d_size);
    outgoing_buffers_[i][rank_].clear();

    size_t n_nodes = d_size / unit_size;

    for (size_t j=0; j<n_nodes; ++j)
      CallbackOnReceiveNode(i, IncomingBuffer() + j * unit_size);
  }
}

void SIMHDGBFS::CallbackOnReceiveNode(int source, const unsigned char *d) {
  static vector<int> values;

  int node = graphs_[rank_]->GenerateNodeIfNotClosedFromBytes(d);

  if (node != -1) {
    IncrementGenerated();
    graphs_[rank_]->State(node, tmp_state_);
    int h = Evaluate(tmp_state_, node, values);

    if (h == -1) {
      IncrementDeadEnds();

      return;
    }

    Push(values, node);
  }
}

vector<int> SIMHDGBFS::ExtractPath(int node) {
  if (node == -1) return vector<int>{-1};

  vector<int> plan;

  while (graphs_[rank_]->Parent(node) != -1) {
    plan.push_back(graphs_[rank_]->Action(node));
    int parent = graphs_[rank_]->Parent(node);
    rank_ = graphs_[rank_]->ParentRank(node);
    node = parent;
  }

  std::reverse(plan.begin(), plan.end());

  return plan;
}

void SIMHDGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;

  double co = static_cast<double>(n_sent_)
    / static_cast<double>(n_sent_or_generated_);

  std::cout << "CO " << co << std::endl;

  for (int i=0; i<world_size_; ++i)
    graphs_[i]->Dump();
}

} // namespace pplanner
