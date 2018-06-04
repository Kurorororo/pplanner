#include "heuristics/rpg.h"

#include <cassert>

using std::unordered_set;
using std::vector;

namespace pplanner {

vector<int> RPG::Plan(const vector<int> &state, unordered_set<int> &helpful,
                      bool common_precond) {
  helpful.clear();
  ConstructGraph(state);
  if (n_layers_ == -1) return vector<int>{-1};

  InitializeGSet();
  auto result = ExtractPlan(common_precond);
  ExtractHelpful(helpful);

  return result;
}

int RPG::PlanCost(const vector<int> &state, bool unit_cost,
                  bool common_precond) {
  ConstructGraph(state);
  if (n_layers_ == -1) return -1;
  InitializeGSet();

  return ExtractCost(unit_cost, common_precond);
}

int RPG::PlanCost(const vector<int> &state, unordered_set<int> &helpful,
                  bool unit_cost, bool common_precond) {
  helpful.clear();
  int h = PlanCost(state, unit_cost, common_precond);
  ExtractHelpful(helpful);

  return h;
}

void RPG::ConstructGraph(const vector<int> &state) {
  Reset();

  for (auto f : state) {
    closed_[f] = true;
    scheduled_facts_.push_back(f);
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

void RPG::Reset() {
  n_layers_ = 0;
  goal_counter_ = problem_->n_goal_facts();
  std::fill(fact_layer_membership_.begin(),
            fact_layer_membership_.end(), -1);
  std::fill(action_layer_membership_.begin(),
            action_layer_membership_.end(), -1);
  std::fill(closed_.begin(), closed_.end(), false);
  std::fill(precondition_counter_.begin(), precondition_counter_.end(), 0);
  scheduled_facts_.clear();
  scheduled_actions_.clear();

  for (auto &g : g_set_)
    g.clear();

  std::fill(selected_.begin(), selected_.end(), false);
  std::fill(is_in_.begin(), is_in_.end(), false);
}

bool RPG::FactLayer() {
  while (!scheduled_facts_.empty()) {
    int f = scheduled_facts_.back();
    scheduled_facts_.pop_back();
    fact_layer_membership_[f] = n_layers_;

    if (problem_->IsGoal(f) && --goal_counter_ == 0)
      return true;

    for (auto o : problem_->PreconditionMap(f)) {
      if (++precondition_counter_[o] == problem_->PreconditionSize(o)) {
        scheduled_actions_.push_back(o);
        is_in_[problem_->ActionId(o)] = true;
      }
    }
  }

  return false;
}

void RPG::ActionLayer() {
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

void RPG::ConstructRRPG(const std::vector<int> &state,
                        const unordered_set<int> &black_list) {
  Reset();

  for (auto f : state) {
    closed_[f] = true;
    scheduled_facts_.push_back(f);
  }

  while (!scheduled_facts_.empty()) {
    RistrictedFactLayer(black_list);
    ActionLayer();
    ++n_layers_;
  }
}

void RPG::RistrictedFactLayer(const unordered_set<int> &black_list) {
  while (!scheduled_facts_.empty()) {
    int f = scheduled_facts_.back();
    scheduled_facts_.pop_back();
    fact_layer_membership_[f] = n_layers_;

    for (auto o : problem_->PreconditionMap(f)) {
      if (++precondition_counter_[o] == problem_->PreconditionSize(o)) {
        int a = problem_->ActionId(o);
        is_in_[a] = true;

        if (black_list.find(a) == black_list.end())
          scheduled_actions_.push_back(o);
      }
    }
  }
}

void RPG::InitializeGSet() {
  g_set_.resize(n_layers_);

  for (auto g : problem_->goal()) {
    g_set_[fact_layer_membership_[g]].push_back(g);
    selected_[g] = true;
  }
}

vector<int> RPG::ExtractPlan(bool common_precond) {
  int m = n_layers_ - 1;
  vector< vector<int> > tmp(m);

  for (int i=m; i>0; --i) {
    std::fill(marked_[0].begin(), marked_[0].end(), false);
    std::fill(marked_[1].begin(), marked_[1].end(), false);

    for (auto g : g_set_[i]) {
      if (marked_[1][g]) continue;
      int o = ExtractAction(i, g, common_precond);
      tmp[i-1].push_back(o);
    }
  }

  vector<int> result;

  for (int i=0; i<m; ++i)
    for (auto a : tmp[i])
      result.push_back(problem_->ActionId(a));

  return result;
}

int RPG::ExtractCost(bool unit_cost, bool common_precond) {
  int m = n_layers_ - 1;
  int h = 0;

  for (int i=m; i>0; --i) {
    std::fill(marked_[0].begin(), marked_[0].end(), false);
    std::fill(marked_[1].begin(), marked_[1].end(), false);

    for (auto g : g_set_[i]) {
      if (marked_[1][g]) continue;
      int o = ExtractAction(i, g, common_precond);

      if (unit_cost)
        ++h;
      else
        h += problem_->ActionCost(o);
    }
  }

  return h;
}

int RPG::ExtractAction(int i, int g, bool common_precond) {
  int o = ChooseAction(g, i, common_precond);

  for (int f : problem_->Precondition(o)) {
    int j = fact_layer_membership_[f];
    selected_[j] = true;

    if (j != 0 && !marked_[0][f])
      g_set_[j].push_back(f);
  }

  int id = problem_->ActionId(o);

  for (auto a : problem_->IdToActions(id)) {
    int f = problem_->Effect(a);
    marked_[0][f] = true;
    marked_[1][f] = true;
  }

  return o;
}

int RPG::ChooseAction(int index, int i, bool common_precond) const {
  int min = -1;
  int argmin = 0;
  int max_n = 0;

  for (auto o : problem_->EffectMap(index)) {
    if (action_layer_membership_[o] != i-1) continue;
    int difficulty = 0;
    int n = 0;

    for (auto p : problem_->Precondition(o)) {
      difficulty += fact_layer_membership_[p];

      if (selected_[p]) ++n;
    }

    if ((difficulty < min || min == -1)
        || (common_precond && difficulty == min && n > max_n)) {
      min = difficulty;
      max_n = n;
      argmin = o;
    }
  }

  assert(-1 != min);

  return argmin;
}
void RPG::ExtractHelpful(unordered_set<int> &helpful) {
  if (n_layers_ < 2) return;

  for (auto g : g_set_[1]) {
    for (auto o : problem_->EffectMap(g))
      if (action_layer_membership_[o] == 0)
        helpful.insert(problem_->ActionId(o));
  }
}

} // namespace pplanner
