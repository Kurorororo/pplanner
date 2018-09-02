#include "search/symmetry_breaking_gbfs.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"
#include "search_graph_factory.h"

namespace pplanner {

using std::make_shared;
using std::unordered_set;
using std::vector;

void SBGBFS::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto opt = pt.get_optional<int>("exhaust")) exhaust_ = true;

  if (auto opt = pt.get_optional<int>("max_expansion")) {
    limit_expansion_ = true;
    max_expansion_ = opt.get();
  }

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  bool keep_cost = false;
  if (auto opt = pt.get_optional<int>("keep_cost")) keep_cost = true;

  bool use_landmark = false;
  if (auto opt = pt.get_optional<int>("landmark")) use_landmark = true;

  bool dump_nodes = false;
  if (auto opt = pt.get_optional<int>("dump_nodes")) dump_nodes = true;

  graph_ = SearchGraphFactory(
      problem_, closed_exponent, keep_cost, use_landmark, dump_nodes);

  vector<std::shared_ptr<Evaluator> > evaluators;
  std::shared_ptr<Evaluator> friend_evaluator = nullptr;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    auto evaluator = EvaluatorFactory(problem_, graph_, friend_evaluator, e);
    evaluators.push_back(evaluator);
    friend_evaluator = evaluator;
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same") {
        preferring_ = evaluators[0];
      } else {
        preferring_ = EvaluatorFactory(problem_, graph_, nullptr,
                                       preferring.get());
      }
    }
  }

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option, evaluators);

  if (auto ram = pt.get_optional<size_t>("ram"))
    graph_->ReserveByRAMSize(ram.get());
  else
    graph_->ReserveByRAMSize(5000000000);

  states_.reserve(graph_->capacity() * graph_->block_size());
  //manager_->Dump();
}

void SBGBFS::SaveState(const std::vector<int> &state) {
  if (states_.size() == states_.capacity()) {
    size_t size = states_.size() / graph_->block_size();
    size_t new_size = 1.2 * size * graph_->block_size();
    states_.reserve(new_size);
  }

  size_t index = states_.size();
  states_.resize(index + graph_->block_size());
  graph_->Pack(state, states_.data() + index);
}

void SBGBFS::RestoreState(int node, std::vector<int> &state) const {
  graph_->Unpack(states_.data() + node * graph_->block_size(), state);
}

vector<int> SBGBFS::InitialExpand() {
  auto state = problem_->initial();
  std::vector<int> canonical;
  manager_->ToCanonical(state, canonical);

  int node = graph_->GenerateNode(-1, -1, canonical);
  SaveState(state);
  ++generated_;

  best_h_ = open_list_->EvaluateAndPush(state, node, true);
  graph_->SetH(node, best_h_);
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  ++evaluated_;

  return state;
}

int SBGBFS::Expand(int node, vector<int> &state, vector<int> &child,
                 vector<int> &applicable, unordered_set<int> &preferred) {
  static std::vector<int> canonical(state);

  if (!graph_->CloseIfNot(node)) return -1;

  ++expanded_;
  RestoreState(node, state);

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    ++dead_ends_;
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

    manager_->ToCanonical(child, canonical);

    int child_node = graph_->GenerateNodeIfNotClosed(o, node, canonical);
    if (child_node == -1) continue;
    ++generated_;

    SaveState(child);
    int h = open_list_->EvaluateAndPush(child, child_node, is_preferred);
    graph_->SetH(child_node, h);
    ++evaluated_;

    if (h == -1) {
      ++dead_ends_;
      continue;
    }

    if (h < best_h_) {
      best_h_ = h;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, "
                << expanded_ << " expanded]" << std::endl;

      if (use_preferred_) open_list_->Boost();
    }
  }

  return -1;
}

int SBGBFS::Search() {
  auto state = InitialExpand();
  vector<int> child(state);
  vector<int> applicable;
  unordered_set<int> preferred;
  int last_goal = -1;

  while (!NoNode()) {
    int node = NodeToExpand();
    int goal = Expand(node, state, child, applicable, preferred);

    if (limit_expansion_ && expanded_ > max_expansion_) return last_goal;

    if (goal != -1 && !exhaust_)
      return goal;
    else if (goal != -1)
      last_goal = goal;
  }

  return last_goal;
}


void SBGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
  std::cout << "Preferred evaluated " << n_preferred_evaluated_ << " state(s)"
            << std::endl;
  std::cout << "Preferred successors " << n_preferreds_ << " state(s)"
            << std::endl;
  double p_p_e = static_cast<double>(n_preferreds_)
    / static_cast<double>(n_preferred_evaluated_);
  std::cout << "Preferreds per state " << p_p_e << std::endl;
  double b_f = static_cast<double>(n_branching_)
    / static_cast<double>(n_preferred_evaluated_);
  std::cout << "Average branching factor " << b_f << std::endl;
  double p_p_b = static_cast<double>(n_preferreds_)
    / static_cast<double>(n_branching_);
  std::cout << "Preferred ratio " << p_p_b  << std::endl;

  graph_->Dump();
}

} // namespace pplanner
