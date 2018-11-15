#include "search/parallel_gbfs_simulator.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"
#include "search_graph_factory.h"
#include "hash/distribution_hash_factory.h"

namespace pplanner {

using std::make_shared;
using std::unordered_set;
using std::vector;

void PGSimulator::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto opt = pt.get_optional<int>("max_expansion"))
    max_expansion_ = opt.get();

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  bool keep_cost = false;
  if (auto opt = pt.get_optional<int>("keep_cost")) keep_cost = true;

  bool use_landmark = false;
  if (auto opt = pt.get_optional<int>("landmark")) use_landmark = true;

  graph_ = SearchGraphFactory(
      problem_, closed_exponent, keep_cost, use_landmark, false);

  std::shared_ptr<Evaluator> friend_evaluator = nullptr;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    auto evaluator = EvaluatorFactory(problem_, graph_, friend_evaluator, e);
    evaluators_.push_back(evaluator);
    friend_evaluator = evaluator;
  }

  n_processes_ = pt.get<int>("n");

  auto open_list_option = pt.get_child("open_list");

  for (int i=0; i<n_processes_; ++i) {
    open_lists_.push_back(OpenListFactory(open_list_option, evaluators_));

    if (use_local_open_)
      local_open_lists_.push_back(
          OpenListFactory(open_list_option, evaluators_));
  }

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram"))
    ram = opt.get();

  graph_->ReserveByRAMSize(ram);

  std::string abstraction = "none";

  if (auto opt = pt.get_optional<std::string>("abstraction"))
    abstraction = opt.get();

  z_hash_ = DistributionHashFactory(problem_, 2886379259, abstraction);
}

int PGSimulator::Search() {
  auto state = InitialEvaluate();
  vector<int> frontier;

  while (!NoNode()) {
    frontier.clear();

    for (int i=0; i<n_processes_; ++i) {
      if (open_lists_[i]->IsEmpty())
        continue;

      frontier.push_back(open_lists_[i]->Pop());
    }

    for (auto node : frontier) {
      int goal = Expand(node, state);

      if (goal != -1 || (max_expansion_ > 0 && expanded_ > max_expansion_))
        return goal;
    }
  }

  return -1;
}

bool PGSimulator::NoNode() const {
  for (int i=0; i<n_processes_; ++i)
    if (!open_lists_[i]->IsEmpty()) return false;

  return true;
}

vector<int> PGSimulator::InitialEvaluate() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);
  ++generated_;

  vector<int> values;
  int h = Evaluate(state, node, values);
  graph_->SetH(node, h);
  best_h_ = h;
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  ++evaluated_;

  uint32_t hash = z_hash_->operator()(state);
  int rank = hash % static_cast<uint32_t>(n_processes_);
  open_lists_[rank]->Push(values, node, false);

  return state;
}


int PGSimulator::Expand(int node, vector<int> &state) {
  static vector<int> applicable;
  static vector<int> values;
  static vector<int> child;

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

    int h = Evaluate(child, child_node, values);

    if (h == -1) {
      ++dead_ends_;
      continue;
    }

    uint32_t hash = z_hash_->operator()(child);
    int to_rank = hash % static_cast<uint32_t>(n_processes_);
    open_lists_[to_rank]->Push(values, child_node, false);
  }

  return -1;
}

int PGSimulator::Evaluate(const vector<int> &state, int node,
                          vector<int> &values) {
  ++evaluated_;
  values.clear();

  for (auto evaluator : evaluators_) {
    int value = evaluator->Evaluate(state, node);

    if (value == -1) {
      ++dead_ends_;
      graph_->SetH(node , value);

      return value;
    }

    values.push_back(value);
  }

  int h = values[0];
  graph_->SetH(node, h);

  if (h < best_h_) {
    best_h_ = h;
    std::cout << "New best heuristic value: " << best_h_ << std::endl;
    std::cout << "[" << evaluated_ << " evaluated, "
              << expanded_ << " expanded]" << std::endl;
  }

  return h;
}

void PGSimulator::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;

  graph_->Dump();
}

} // namespace pplanner
