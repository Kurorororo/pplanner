#include "search/gbfs.h"

#include <cassert>

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"

namespace pplanner {

using std::unordered_set;
using std::vector;

void GBFS::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  graph_ = std::unique_ptr<SearchGraph>(
      new SearchGraph(*problem_, closed_exponent));

  vector<std::shared_ptr<Evaluator> > evaluators;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    evaluators.push_back(EvaluatorFactory(problem_, e));
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;
    preferring_ = EvaluatorFactory(problem_, preferring.get());
  }

  int n_boost = 0;

  if (auto n_boost_opt = pt.get_optional<int>("boost"))
    n_boost = n_boost_opt.get();

  if (auto tie_breaking = pt.get_optional<std::string>("tie-breaking")) {
    open_list_ = OpenListFactory(tie_breaking.get(), evaluators, use_preferred_,
                                 n_boost);
  } else {
    open_list_ = OpenListFactory("fifo", evaluators, use_preferred_, n_boost);
  }

  if (auto ram = pt.get_optional<size_t>("ram"))
    graph_->ReserveByRAMSize(ram.get());
  else
    graph_->ReserveByRAMSize(5000000000);
}

int GBFS::Search() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(state, -1, -1, true);
  ++generated_;

  int best_h = open_list_->EvaluateAndPush(state, node, true);
  std::cout << "Initial heuristic value: " << best_h << std::endl;
  ++evaluated_;

  vector<int> child(state);
  vector<int> applicable;
  unordered_set<int> preferred;

  while (!open_list_->IsEmpty()) {
    int node = open_list_->Pop();
    ++expanded_;

    if (graph_->GetStateAndClosed(node, state) != -1) continue;
    graph_->Close(node);

    if (problem_->IsGoal(state)) return node;

    generator_->Generate(state, applicable);

    if (applicable.empty()) {
      ++dead_ends_;
      continue;
    }

    if (use_preferred_) {
      preferring_->Evaluate(state, node, applicable, preferred);
      n_preferreds_ += preferred.size();
    }

    ++n_preferred_evaluated_;

    for (auto o : applicable) {
      child = state;
      problem_->ApplyEffect(o, child);

      bool is_preferred = use_preferred_ && preferred.find(o) != preferred.end();

      int child_node = graph_->GenerateNodeIfNotClosed(child, node, o,
                                                       is_preferred);
      if (child_node == -1) continue;
      ++generated_;

      int h = open_list_->EvaluateAndPush(child, child_node, is_preferred);
      ++evaluated_;

      if (h == -1) {
        ++dead_ends_;
        continue;
      }

      if (is_preferred) ++n_preferred_states_;

      if (h < best_h) {
        best_h = h;
        std::cout << "New best heuristic value: " << best_h << std::endl;
        std::cout << "[" << evaluated_ << " evaluated, "
                  << expanded_ << " expanded]" << std::endl;

        if (use_preferred_) open_list_->Boost();
      }
    }
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
  std::cout << "Preferred operators " << n_preferreds_ << std::endl;
  std::cout << "Preferred successors " << n_preferred_states_ << " state(s)"
            << std::endl;
  double p_p_e = static_cast<double>(n_preferreds_)
    / static_cast<double>(n_preferred_evaluated_);
  std::cout << "Preferreds per state " << p_p_e << std::endl;
}

} // namespace pplanner
