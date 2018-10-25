#include "search/lazy_gbfs.h"

#include <cassert>

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

void LazyGBFS::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  if (auto use_landmark = pt.get_optional<int>("landmark"))
    graph_ = make_shared<SearchGraphWithLandmarks>(problem_, closed_exponent);
  else
    graph_ = make_shared<SearchGraph>(problem_, closed_exponent);

  std::shared_ptr<Evaluator> friend_evaluator = nullptr;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    auto evaluator = EvaluatorFactory(problem_, graph_, friend_evaluator, e);
    evaluators_.push_back(evaluator);
    friend_evaluator = evaluator;
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same") {
        same_ = true;
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
}

int LazyGBFS::Search() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);
  ++generated_;

  vector<int> child(state);
  vector<int> applicable;
  unordered_set<int> preferred;

  int best_h = -1;

  while (!open_list_->IsEmpty() || best_h == -1) {
    if (best_h != -1) node = open_list_->Pop();
    if (!graph_->CloseIfNot(node)) continue;

    ++expanded_;
    graph_->Expand(node, state);

    if (problem_->IsGoal(state)) return node;

    generator_->Generate(state, applicable);

    if (applicable.empty()) {
      ++dead_ends_;
      continue;
    }

    int h = Evaluate(state, node, applicable, preferred);
    ++evaluated_;

    if (h == -1) {
      ++dead_ends_;
      continue;
    }

    if (h < best_h || best_h == -1) {
      best_h = h;
      std::cout << "New best heuristic value: " << best_h << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, " << expanded_
                << " expanded]" << std::endl;

      if (use_preferred_) open_list_->Boost();
    }

    for (auto o : applicable) {
      child = state;
      problem_->ApplyEffect(o, child);

      bool is_preferred = use_preferred_
        && preferred.find(o) != preferred.end();

      int child_node = graph_->GenerateNodeIfNotClosed(o, node, state, child);
      if (child_node == -1) continue;
      ++generated_;

      open_list_->Push(values_, child_node, is_preferred);

      if (is_preferred) {
        is_preferred_action_[o] = true;
        ++n_preferreds_;
      }

      ++n_branching_;
    }
  }

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

      int value = e->Evaluate(state, node);

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
  double b_f = static_cast<double>(n_branching_)
    / static_cast<double>(evaluated_);
  std::cout << "Average branching factor " << b_f << std::endl;
  double p_p_b = static_cast<double>(n_preferreds_)
    / static_cast<double>(n_branching_);
  std::cout << "Preferred ratio " << p_p_b  << std::endl;

  DumpPreferringMetrics();
}

void LazyGBFS::DumpPreferringMetrics() const {
  vector<bool> in_plan(problem_->n_actions(), false);

  for (auto a : plan_)
    in_plan[a] = true;

  int tp = 0;
  int fn = 0;
  int fp = 0;
  int tn = 0;

  for (int i=0, n=problem_->n_actions(); i<n; ++i) {
    if (in_plan[i] && is_preferred_action_[i]) ++tp;
    if (in_plan[i] && !is_preferred_action_[i]) ++fn;
    if (!in_plan[i] && is_preferred_action_[i]) ++fp;
    if (!in_plan[i] && !is_preferred_action_[i]) ++tn;
  }

  double accuracy = static_cast<double>(tp + tn)
    / static_cast<double>(tp + fn + fp + tn);

  double precision = 0.0;

  if (tp + fp != 0)
    precision = static_cast<double>(tp) / static_cast<double>(tp + fp);

  double recall = 0.0;

  if (tp + fn != 0)
    recall = static_cast<double>(tp) / static_cast<double>(tp + fn);

  double f = 0.0;

  if (precision + recall > 1.0e-14)
    f = (2.0 * recall * precision) / (recall + precision);

  std::cout << std::endl;
  std::cout << "Operator TP " << tp << std::endl;
  std::cout << "Operator FN " << fn << std::endl;
  std::cout << "Operator FP " << fp << std::endl;
  std::cout << "Operator TN " << tn << std::endl;
  std::cout << "Operator Accuracy " << accuracy << std::endl;
  std::cout << "Operator Precision " << precision << std::endl;
  std::cout << "Operator Recall " << recall << std::endl;
  std::cout << "Operator F " << f << std::endl;

  double step = plan_.size();
  double normalized_f = f / step;

  std::cout << "Normalized F " << normalized_f << std::endl;
  std::cout << std::endl;
}

} // namespace pplanner
