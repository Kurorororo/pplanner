#include "search/simhdgbfs1.h"

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

void SIMHDGBFS1::Init(const boost::property_tree::ptree &pt) {
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
  }

  auto null_evaluator = EvaluatorFactory(
      problem_, graphs_[0], nullptr, evaluator_names[0]);

  if (auto opt = pt.get_optional<int>("local_open"))
    use_local_open_ = true;

  for (int i=0; i<world_size_; ++i) {
    rank_ = i;
    std::shared_ptr<Evaluator> friend_evaluator = nullptr;

    for (auto e : evaluator_names) {
      auto evaluator = EvaluatorFactory(
          problem_, graphs_[rank_], friend_evaluator, e);
      evaluators_[rank_].push_back(evaluator);
      friend_evaluator = evaluator;
    }

    open_lists_.push_back(
        OpenListFactory(open_list_option, evaluators_[rank_]));

    if (use_local_open_)
      local_open_lists_.push_back(
          OpenListFactory(open_list_option, evaluators_[rank_]));

    graphs_[rank_]->ReserveByRAMSize(ram);
  }

  if (auto opt = pt.get_optional<int>("take"))
    take_ = opt.get();

  std::string abstraction = "none";

  if (auto opt = pt.get_optional<std::string>("abstraction"))
    abstraction = opt.get();

  z_hash_ = DistributionHashFactory(problem_, 2886379259, abstraction);

  best_values_.resize(world_size_);

  if (auto opt = pt.get_optional<int>("delay"))
    delay_ = opt.get();

  outgoing_buffers_.resize(world_size_,
                           vector<vector<unsigned char> >(world_size_));
  incoming_buffers_.resize(world_size_,
                           vector<vector<unsigned char> >(delay_));
}

int SIMHDGBFS1::Search() {
  auto state = InitialEvaluate();

  while (true) {
    for (int i=0; i<world_size_; ++i) {
      rank_ = i;
      CommunicateNodes();
    }

    ++delay_index_;

    if (delay_index_ == delay_)
      delay_index_ = 0;

    for (int i=0; i<world_size_; ++i) {
      rank_ = i;
      int node = -1;

      while (node == -1 && !NoNode()) {
        node = Pop();

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

vector<int> SIMHDGBFS1::InitialEvaluate() {
  auto state = problem_->initial();
  uint32_t world_size = static_cast<uint32_t>(world_size_);
  rank_ = z_hash_->operator()(state) % world_size;

  int node = graphs_[rank_]->GenerateNode(-1, -1, state, -1);

  IncrementGenerated();

  int h = -1;

  if (use_local_open_)
    h = local_open_lists_[rank_]->EvaluateAndPush(state, node, true);
  else
    h = open_lists_[rank_]->EvaluateAndPush(state, node, true);

  graphs_[rank_]->SetH(node, h);
  set_best_h(h);
  std::cout << "Initial heuristic value: " << best_h() << std::endl;
  ++evaluated_;

  return state;
}

int SIMHDGBFS1::Expand(int node, vector<int> &state) {
  static vector<int> applicable;
  static vector<vector<int> > state_array;
  static vector<vector<uint32_t> > packed_array;
  static vector<int> actione_array;
  static vector<uint32_t> hash_array;
  static vector<vector<int> > value_array;

  ++expanded_;
  graphs_[rank_]->Expand(node, state);

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    IncrementDeadEnds();
    return -1;
  }

  n_branching_ += applicable.size();

  state_array.resize(applicable.size());
  packed_array.resize(applicable.size());
  actione_array.resize(applicable.size());
  hash_array.resize(applicable.size());
  value_array.resize(applicable.size());

  int index = 0;
  int arg_min = -1;

  for (auto o : applicable) {
    auto &child = state_array[index];
    child = state;
    problem_->ApplyEffect(o, child);

    auto &packed = packed_array[index];
    packed.resize(graphs_[rank_]->block_size());
    std::fill(packed.begin(), packed.end(), 0);
    auto &hash = hash_array[index];

    int closed = graphs_[rank_]->GetClosed(
        o, node, state, child, packed.data(), &hash);

    if (closed != -1) continue;

    auto &values = value_array[index];
    int h = Evaluate(child, -(index + 1), node, values);

    if (h == -1) {
      IncrementDeadEnds();
      continue;
    }

    if (arg_min == -1 || values < value_array[arg_min])
      arg_min = index;

    actione_array[index] = o;
    index++;
  }

  if (index == 0) return -1;

  int node_to_keep = -1;

  if (take_ == 0 && (NoNode() || value_array[arg_min] < MinimumValues()))
    node_to_keep = arg_min;

  if (take_ == 1 && value_array[arg_min] < best_values_[rank_])
    node_to_keep = arg_min;

  for (int i=0; i<index; ++i) {
    ++n_sent_or_generated_;

    auto &child = state_array[i];
    auto &values = value_array[i];
    int action = actione_array[i];

    int to_rank = -1;

    if (i != node_to_keep) {
      uint32_t hash = z_hash_->operator()(child);
      to_rank = hash % static_cast<uint32_t>(world_size_);
    }

    if (to_rank != -1 && to_rank != rank_) {
      if (i == arg_min && (NoNode() || values < MinimumValues()))
        ++n_sent_next_;

      unsigned char *buffer = ExtendOutgoingBuffer(
          to_rank, values.size() * sizeof(int) + node_size());
      memcpy(buffer, values.data(), values.size() * sizeof(int));
      graphs_[rank_]->BufferEvaluatedNode(
          i, action, node, hash_array[i], packed_array[i].data(),
          buffer + values.size() * sizeof(int));
      ++n_sent_;
    }

    if (i == node_to_keep || to_rank == rank_) {
      if (i == arg_min && (NoNode() || values < MinimumValues()))
        ++n_pushed_next_;

      int child_node = graphs_[rank_]->GenerateEvaluatedNode(
          i, action, node, hash_array[i], packed_array[i].data(), rank_);

      Push(values, child_node, i == node_to_keep);

      IncrementGenerated();
    }
  }

  return -1;
}

int SIMHDGBFS1::Evaluate(const vector<int> &state, int node, int parent,
                         vector<int> &values) {
  ++evaluated_;

  values.clear();

  for (auto evaluator : evaluators_[rank_]) {
    int value = evaluator->Evaluate(state, node, parent);

    if (value == -1) {
      IncrementDeadEnds();

      return value;
    }

    values.push_back(value);
  }

  return values[0];
}


void SIMHDGBFS1::Push(std::vector<int> &values, int node, bool is_local) {
  int h = values[0];

  if (use_local_open_ && (is_local || h == 0))
    local_open_lists_[rank_]->Push(values, node, false);
  else
    open_lists_[rank_]->Push(values, node, false);

  graphs_[rank_]->SetH(node, values[0]);

  if (best_values_[rank_].empty()
      || ((!use_local_open_ || is_local) && values < best_values_[rank_]))
    best_values_[rank_] = values;

  if (best_h() == -1 || h < best_h()) {
    set_best_h(h);

   std::cout << "New best heuristic value: " << best_h() << std::endl;
   std::cout << "[" << evaluated_ << " evaluated, "
             << expanded_ << " expanded]" << std::endl;
  }
}

void SIMHDGBFS1::CommunicateNodes() {
  static vector<int> values(n_evaluators());

  size_t d_size = 0;

  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || outgoing_buffers_[i][rank_].empty()) continue;
    d_size += outgoing_buffers_[i][rank_].size();
  }

  incoming_buffers_[rank_][delay_index_].resize(d_size);
  size_t size_sum = 0;

  for (int i=0; i<world_size_; ++i) {
    if (i == rank_ || outgoing_buffers_[i][rank_].empty()) continue;

    size_t size = outgoing_buffers_[i][rank_].size();
    memcpy(incoming_buffers_[rank_][delay_index_].data() + size_sum,
           outgoing_buffers_[i][rank_].data(), size);
    outgoing_buffers_[i][rank_].clear();
    size_sum += size;
  }

  size_t unit_size = node_size() + values.size() * sizeof(int);
  size_t n_nodes = incoming_buffers_[rank_][delay_ - delay_index_ - 1].size();
  n_nodes /= unit_size;
  auto buffer = incoming_buffers_[rank_][delay_ - delay_index_ - 1].data();

  for (size_t i=0; i<n_nodes; ++i) {
    memcpy(values.data(), buffer, values.size() * sizeof(int));
    buffer += values.size() * sizeof(int);
    int node = graphs_[rank_]->GenerateNodeIfNotClosedFromBytes(buffer);

    if (node != -1) {
      IncrementGenerated();
      Push(values, node, false);
    }

    buffer += node_size();
  }
}

vector<int> SIMHDGBFS1::ExtractPath(int node) {
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

void SIMHDGBFS1::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;

  double co = static_cast<double>(n_sent_)
    / static_cast<double>(n_sent_or_generated_);

  std::cout << "CO " << co << std::endl;

  std::cout << "Local node to expand " << n_pushed_next_ << std::endl;
  std::cout << "Remote node to expand " << n_sent_next_ << std::endl;

  double pnpe = static_cast<double>(n_pushed_next_)
    / static_cast<double>(n_pushed_next_ + n_sent_next_);
  double snpe = static_cast<double>(n_sent_next_)
    / static_cast<double>(n_pushed_next_ + n_sent_next_);

  std::cout << "Local node to expand ratio " << pnpe << std::endl;
  std::cout << "Remote node to expand ratio " << snpe << std::endl;

  for (int i=0; i<world_size_; ++i)
    graphs_[i]->Dump();
}

} // namespace pplanner
