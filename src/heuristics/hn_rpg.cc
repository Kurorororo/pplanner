#include "heuristics/hn_rpg.h"

#include <cassert>

using std::unordered_set;
using std::vector;

namespace pplanner {

vector<int> HNRPG::Plan(const vector<int> &state, unordered_set<int> &helpful) {
  ConstructGraph(state);
  if (n_layers_ == -1) return vector<int>{-1};

  InitializeGSet();
  auto result = ExtractPlan();
  ExtractHelpful(helpful);

  return result;
}

int HNRPG::PlanCost(const vector<int> &state, bool unit_cost) {
  ConstructGraph(state);
  if (n_layers_ == -1) return -1;
  InitializeGSet();

  return ExtractCost(unit_cost);
}

int HNRPG::PlanCost(const vector<int> &state, unordered_set<int> &helpful,
                    bool unit_cost) {
  int h = PlanCost(state, unit_cost);
  ExtractHelpful(helpful);

  return h;
}

void HNRPG::ConstructGraph(const vector<int> &state) {
  SetUp();

  for (auto f : state) {
    closed_[f] = true;
    scheduled_facts_.push_back(f);
  }

  for (auto o : problem_->NoPreconditions()) {
    is_applicable_[o] = true;
    scheduled_actions_.push_back(o);
  }

  while (!scheduled_facts_.empty()) {
    if (FactLayer()) {
      ++n_layers_;
      return;
    }

    ActionLayer();
    ++n_layers_;
  }

  n_layers_ = -1;
}

void HNRPG::SetUp() {
  n_layers_ = 0;
  goal_counter_ = problem_->n_goal_facts();
  std::fill(fact_layer_membership_.begin(), fact_layer_membership_.end(), -1);
  std::fill(action_layer_membership_.begin(), action_layer_membership_.end(),
            -1);
  std::fill(closed_.begin(), closed_.end(), false);
  std::fill(is_applicable_.begin(), is_applicable_.end(), false);

  for (int i = 0, n = problem_->n_actions(); i < n; ++i)
    precondition_counter_[i] = problem_->PreconditionSize(i);

  scheduled_facts_.clear();
  scheduled_actions_.clear();

  for (auto &g : g_set_) g.clear();
}

bool HNRPG::FactLayer() {
  while (!scheduled_facts_.empty()) {
    int f = scheduled_facts_.back();
    scheduled_facts_.pop_back();
    fact_layer_membership_[f] = n_layers_;

    if (problem_->IsGoal(f) && --goal_counter_ == 0) return true;

    for (auto o : problem_->PreconditionMap(f)) {
      if (--precondition_counter_[o] == 0) {
        is_applicable_[problem_->ActionId(o)] = true;
        scheduled_actions_.push_back(o);
      }
    }
  }

  return false;
}

void HNRPG::ActionLayer() {
  while (!scheduled_actions_.empty()) {
    int o = scheduled_actions_.back();
    scheduled_actions_.pop_back();
    action_layer_membership_[o] = n_layers_;

    int f = problem_->Effect(o);

    if (!closed_[f]) {
      closed_[f] = true;
      scheduled_facts_.push_back(f);
    }
  }
}

void HNRPG::ConstructRRPG(const vector<int> &state,
                          const vector<bool> &black_list) {
  SetUp();

  for (auto f : state) {
    closed_[f] = true;
    scheduled_facts_.push_back(f);
  }

  for (auto o : problem_->NoPreconditions()) {
    is_applicable_[o] = true;
    if (!black_list[o]) scheduled_actions_.push_back(o);
  }

  while (!scheduled_facts_.empty()) {
    RistrictedFactLayer(black_list);
    ActionLayer();
    ++n_layers_;
  }
}

void HNRPG::RistrictedFactLayer(const vector<bool> &black_list) {
  while (!scheduled_facts_.empty()) {
    int f = scheduled_facts_.back();
    scheduled_facts_.pop_back();
    fact_layer_membership_[f] = n_layers_;

    for (auto o : problem_->PreconditionMap(f)) {
      if (--precondition_counter_[o] == 0) {
        is_applicable_[problem_->ActionId(o)] = true;
        if (!black_list[o]) scheduled_actions_.push_back(o);
      }
    }
  }
}

void HNRPG::InitializeGSet() {
  g_set_.resize(n_layers_);

  for (auto g : problem_->goal())
    g_set_[fact_layer_membership_[g]].push_back(g);
}

vector<int> HNRPG::ExtractPlan() {
  int m = n_layers_ - 1;
  vector<vector<int> > tmp(m);

  for (int i = m; i > 0; --i) {
    std::fill(marked_[0].begin(), marked_[0].end(), false);
    std::fill(marked_[1].begin(), marked_[1].end(), false);

    for (auto g : g_set_[i]) {
      if (marked_[1][g]) continue;
      int o = ExtractAction(i, g);
      tmp[i - 1].push_back(o);
    }
  }

  vector<int> result;

  for (int i = 0; i < m; ++i)
    for (auto a : tmp[i]) result.push_back(problem_->ActionId(a));

  return result;
}

int HNRPG::ExtractCost(bool unit_cost) {
  int m = n_layers_ - 1;
  int h = 0;

  for (int i = m; i > 0; --i) {
    std::fill(marked_[0].begin(), marked_[0].end(), false);
    std::fill(marked_[1].begin(), marked_[1].end(), false);

    for (auto g : g_set_[i]) {
      if (marked_[1][g]) continue;
      int o = ExtractAction(i, g);

      if (unit_cost)
        ++h;
      else
        h += problem_->ActionCost(o);
    }
  }

  return h;
}

int HNRPG::ExtractAction(int i, int g) {
  int o = ChooseAction(g, i);

  for (int f : problem_->Precondition(o)) {
    int j = fact_layer_membership_[f];

    if (j != 0 && !marked_[0][f]) g_set_[j].push_back(f);
  }

  int id = problem_->ActionId(o);

  for (auto a : problem_->IdToActions(id)) {
    if (a == o || !problem_->IsConditional(a)) {
      int f = problem_->Effect(a);
      marked_[0][f] = true;
      marked_[1][f] = true;
    }
  }

  return o;
}

int HNRPG::ChooseAction(int index, int i) const {
  int min = -1;
  int argmin = 0;

  for (auto o : problem_->EffectMap(index)) {
    if (action_layer_membership_[o] != i - 1) continue;
    int difficulty = 0;

    for (auto p : problem_->Precondition(o))
      difficulty += fact_layer_membership_[p];

    if (difficulty < min || min == -1) {
      min = difficulty;
      argmin = o;
    }
  }

  assert(-1 != min);

  return argmin;
}
void HNRPG::ExtractHelpful(unordered_set<int> &helpful) {
  helpful.clear();

  if (n_layers_ < 2) return;

  for (auto g : g_set_[1]) {
    for (auto o : problem_->EffectMap(g))
      if (action_layer_membership_[o] == 0)
        helpful.insert(problem_->ActionId(o));
  }
}

void HNRPG::DisjunctiveHelpful(const vector<int> &state,
                               const vector<int> &disjunctive_goal,
                               unordered_set<int> &helpful, bool unit_cost) {
  int f = ConstructDisjunctiveRPG(state, disjunctive_goal, helpful);

  if (f == -1) return;

  g_set_.resize(n_layers_);
  g_set_[fact_layer_membership_[f]].push_back(f);
  ExtractCost(unit_cost);
  ExtractHelpful(helpful);
}

int HNRPG::ConstructDisjunctiveRPG(const vector<int> &state,
                                   const vector<int> &disjunctive_goals,
                                   unordered_set<int> &helpful) {
  SetUp();
  std::fill(is_disjunctive_goal_.begin(), is_disjunctive_goal_.end(), false);

  for (auto g : disjunctive_goals) is_disjunctive_goal_[g] = true;

  for (auto f : state) {
    closed_[f] = true;
    scheduled_facts_.push_back(f);
  }

  for (auto o : problem_->NoPreconditions()) {
    is_applicable_[o] = true;
    scheduled_actions_.push_back(o);
  }

  while (!scheduled_facts_.empty()) {
    int f = DisjunctiveFactLayer();

    if (f != -1) {
      ++n_layers_;
      return f;
    }

    ActionLayer();
    ++n_layers_;
  }

  return -1;
}

int HNRPG::DisjunctiveFactLayer() {
  while (!scheduled_facts_.empty()) {
    int f = scheduled_facts_.back();
    scheduled_facts_.pop_back();
    fact_layer_membership_[f] = n_layers_;

    if (is_disjunctive_goal_[f]) return f;

    for (auto o : problem_->PreconditionMap(f)) {
      if (--precondition_counter_[o] == 0) {
        is_applicable_[problem_->ActionId(o)] = true;
        scheduled_actions_.push_back(o);
      }
    }
  }

  return -1;
}

}  // namespace pplanner
