#include "search/bts_gbfs.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"
#include "search_graph_factory.h"

namespace pplanner {

using std::make_shared;
using std::size_t;
using std::unordered_set;
using std::vector;

void BTSGBFS::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  bool keep_cost = false;
  if (auto opt = pt.get_optional<int>("keep_cost")) keep_cost = true;

  bool use_landmark = false;
  if (auto opt = pt.get_optional<int>("landmark")) use_landmark = true;

  graph_ = SearchGraphFactory(problem_, closed_exponent, keep_cost,
                              use_landmark, true);

  BOOST_FOREACH (const boost::property_tree::ptree::value_type &child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    auto evaluator = EvaluatorFactory(problem_, e, nullptr, graph_);
    evaluators_.push_back(evaluator);
  }

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option);

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram")) ram = opt.get();

  graph_->ReserveByRAMSize(ram);
}

vector<int> BTSGBFS::InitialExpand() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);
  ++generated_;

  std::vector<int> values;
  best_h_ = Evaluate(evaluators_, state, node, values);
  hwm_ = best_h_;
  graph_->SetH(node, best_h_);
  open_list_->Push(values, node, false);
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  ++evaluated_;

  return state;
}

int BTSGBFS::Expand(vector<int> &state, vector<int> &child,
                    vector<int> &applicable, bool plan_found) {
  static vector<int> values;

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
    problem_->ApplyEffect(o, state, child);
    int child_node = graph_->GenerateNodeIfNotClosed(o, node, state, child);
    if (child_node == -1) continue;

    ++generated_;

    int h = Evaluate(evaluators_, child, child_node, values);
    graph_->SetH(child_node, h);
    ++evaluated_;

    if (h == -1) {
      ++dead_ends_;
      continue;
    }

    if (!plan_found && h > hwm_) hwm_ = h;

    if (plan_found && h > hwm_) continue;

    open_list_->Push(values, child_node, false);

    if (h < best_h_) {
      best_h_ = h;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, " << expanded_
                << " expanded]" << std::endl;
    }
  }

  return -1;
}

int BTSGBFS::Search() {
  auto state = InitialExpand();
  vector<int> child(state);
  vector<int> applicable;
  int first_goal = -1;

  while (!open_list_->IsEmpty()) {
    int goal = Expand(state, child, applicable, first_goal != -1);

    if (goal != -1) graph_->MarkGoal(goal);

    if (goal != -1 && first_goal == -1) first_goal = goal;
  }

  return first_goal;
}

void BTSGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;

  graph_->Dump();
}

}  // namespace pplanner
