#include "search/lazy_gbfs.h"

#include <cassert>

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"

namespace pplanner {

using std::unordered_set;
using std::vector;

void LazyGBFS::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  graph_ = std::unique_ptr<SearchGraph>(
      new SearchGraph(*problem_, closed_exponent));

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    evaluators_.push_back(EvaluatorFactory(problem_, e));
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same")
        same_ = true;
      else
        preferring_ = EvaluatorFactory(problem_, preferring.get());
    }
  }

  int n_boost = 0;

  if (auto n_boost_opt = pt.get_optional<int>("boost"))
    n_boost = n_boost_opt.get();

  if (auto tie_breaking = pt.get_optional<std::string>("tie-breaking")) {
    open_list_ = OpenListFactory(tie_breaking.get(), evaluators_, use_preferred_,
                                 n_boost);
  } else {
    open_list_ = OpenListFactory("fifo", evaluators_, use_preferred_, n_boost);
  }

  if (auto ram = pt.get_optional<size_t>("ram"))
    graph_->ReserveByRAMSize(ram.get());
  else
    graph_->ReserveByRAMSize(5000000000);
}

int LazyGBFS::Search() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(state, -1, -1, true);
  ++generated_;
  ++expanded_;

  vector<int> child(state);
  vector<int> applicable;
  unordered_set<int> preferred;

  int best_h = -1;

  do {
    if (graph_->GetStateAndClosed(node, state) != -1){
      node = open_list_->Pop();
      continue;
    }

    graph_->Close(node);

    int h = Evaluate(state, node, applicable, preferred);
    ++evaluated_;

    if (h == -1) {
      ++dead_ends_;
      node = open_list_->Pop();
      ++expanded_;
      continue;
    }

    if (h < best_h || best_h == -1) {
      best_h = h;
      std::cout << "New best heuristic value: " << best_h << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, " << expanded_
                << " expanded]" << std::endl;

      if (use_preferred_) open_list_->Boost();
    }

    if (problem_->IsGoal(state)) return node;

    generator_->Generate(state, applicable);

    if (applicable.empty()) {
      ++dead_ends_;
      node = open_list_->Pop();
      ++expanded_;
      continue;
    }

    for (auto o : applicable) {
      child = state;
      problem_->ApplyEffect(o, child);

      bool is_preferred = use_preferred_ && preferred.find(o) != preferred.end();
      int child_node = graph_->GenerateNode(child, node, o, is_preferred);
      ++generated_;

      open_list_->Push(values_, child_node, is_preferred);

      if (is_preferred) ++n_preferreds_;
    }

    node = open_list_->Pop();
    ++expanded_;
  } while (!open_list_->IsEmpty());

  return -1;
}

int LazyGBFS::Evaluate(const vector<int> &state, int node,
                       const vector<int> &applicable,
                       unordered_set<int> &preferred) {
  values_.clear();

  if (use_preferred_) {
    bool first = true;

    for (auto e : evaluators_) {
      if (same_ && first) {
        int value = e->Evaluate(state, node, applicable, preferred);

        if (value == -1) return value;

        values_.push_back(value);
        first = false;

        continue;
      }

      int value = e->Evaluate(state, node, applicable, preferred);

      if (value == -1) return value;

      values_.push_back(value);
    }

    if (!same_) preferring_->Evaluate(state, node);

    return values_[0];
  }

  for (auto e : evaluators_)
    values_.push_back(e->Evaluate(state, node));

  return values_[0];
}

void LazyGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
  std::cout << "Preferred successors " << n_preferreds_ << " state(s)"
            << std::endl;
  double p_p_e = static_cast<double>(n_preferreds_)
    / static_cast<double>(evaluated_);
  std::cout << "Preferreds per state " << p_p_e << std::endl;
}

} // namespace pplanner
