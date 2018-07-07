#include "search/gbfs.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"
#include "search_graph/search_graph_with_landmarks.h"

namespace pplanner {

using std::make_shared;
using std::unordered_set;
using std::vector;

void GBFS::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  if (auto use_landmark = pt.get_optional<int>("landmark"))
    graph_ = make_shared<SearchGraphWithLandmarks>(problem_, closed_exponent);
  else
    graph_ = make_shared<SearchGraph>(problem_, closed_exponent);

  vector<std::shared_ptr<Evaluator> > evaluators;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    evaluators.push_back(EvaluatorFactory(problem_, graph_, e));
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same")
        preferring_ = evaluators[0];
      else
        preferring_ = EvaluatorFactory(problem_, graph_, preferring.get());
    }
  }

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option, evaluators);

  if (auto ram = pt.get_optional<size_t>("ram"))
    graph_->ReserveByRAMSize(ram.get());
  else
    graph_->ReserveByRAMSize(5000000000);
}

vector<int> GBFS::InitialExpand() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);
  ++generated_;

  best_h_ = open_list_->EvaluateAndPush(state, node, true);
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  ++evaluated_;

  return state;
}

int GBFS::Expand(int node, vector<int> &state, vector<int> &child,
                 vector<int> &applicable, unordered_set<int> &preferred) {
  if (!graph_->CloseIfNot(node)) return -1;

  ++expanded_;
  graph_->State(node, state);

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

    int child_node = graph_->GenerateNodeIfNotClosed(o, node, state, child);
    if (child_node == -1) continue;
    ++generated_;

    int h = open_list_->EvaluateAndPush(child, child_node, is_preferred);
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

int GBFS::Search() {
  auto state = InitialExpand();
  vector<int> child(state);
  vector<int> applicable;
  unordered_set<int> preferred;

  while (!NoNode()) {
    int node = NodeToExpand();
    int goal = Expand(node, state, child, applicable, preferred);

    if (goal != -1) return goal;
  }

  return -1;
}


void GBFS::DumpStatistics() const {
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
}

} // namespace pplanner
