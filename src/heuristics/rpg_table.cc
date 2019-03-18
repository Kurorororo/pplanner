#include "rpg_table.h"

namespace pplanner {

using std::unordered_set;
using std::vector;

int RPGTable::PlanCost(const vector<int> &state, unordered_set<int> &helpful) {
  int additive_h = AdditiveCost(state);
  if (additive_h == -1) return -1;

  std::fill(plan_set_.begin(), plan_set_.end(), false);
  std::fill(marked_.begin(), marked_.end(), false);
  helpful.clear();

  for (auto g : r_problem_->goal()) {
    SetPlan(g, helpful);
  }

  int h = 0;

  for (int i = 0, n = plan_set_.size(); i < n; ++i) {
    if (plan_set_[i]) h += unit_cost_ ? 1 : problem_->ActionCost(i);
  }

  return h;
}

int RPGTable::AdditiveCost(const vector<int> &state) {
  SetUp(state);
  GeneralizedDijkstra(state);

  int h = 0;

  for (auto g : r_problem_->goal()) {
    int cost = prop_cost_[g];
    if (cost == -1) return cost;
    h += cost;
  }

  return h;
}

int RPGTable::HmaxCost(const vector<int> &state) {
  SetUp(state);
  GeneralizedDijkstra(state, true);

  int h = 0;

  for (auto g : r_problem_->goal()) {
    int cost = prop_cost_[g];
    if (cost == -1) return cost;
    h = std::max(h, cost);
  }

  return h;
}

void RPGTable::SetPlan(int g, unordered_set<int> &helpful) {
  if (marked_[g]) return;
  marked_[g] = true;

  int unary_a = best_support_[g];
  if (unary_a == -1) return;
  bool is_helpful = true;

  for (auto p : r_problem_->Precondition(unary_a)) {
    SetPlan(p, helpful);
    if (best_support_[p] != -1) is_helpful = false;
  }

  int a = r_problem_->ActionId(unary_a);
  plan_set_[a] = true;

  if (is_helpful) {
    helpful.insert(a);

    if (more_helpful_)
      for (auto supporter : supporters_[g])
        helpful.insert(r_problem_->ActionId(supporter));
  }
}

void RPGTable::GeneralizedDijkstra(const vector<int> &state, bool hmax) {
  for (int i = 0, n = r_problem_->n_actions(); i < n; ++i) {
    if (precondition_counter_[i] == 0) {
      is_applicable_[r_problem_->ActionId(i)] = true;
      MayPush(r_problem_->Effect(i), i);
    }
  }

  for (auto f : state) {
    prop_cost_[f] = 0;
    q_->Push(0, f);
  }

  while (!q_->IsEmpty()) {
    int c = q_->MinimumValue();
    int f = q_->Pop();

    assert(prop_cost_[f] != -1);

    if (prop_cost_[f] < c) continue;
    if (r_problem_->IsGoal(f) && --goal_counter_ == 0) return;

    for (auto a : r_problem_->PreconditionMap(f)) {
      if (hmax)
        op_cost_[a] = std::max(r_problem_->ActionCost(a) + c, op_cost_[a]);
      else
        op_cost_[a] += c;

      if (--precondition_counter_[a] == 0) {
        is_applicable_[r_problem_->ActionId(a)] = true;
        MayPush(r_problem_->Effect(a), a);
      }
    }
  }
}

void RPGTable::SetUp(const vector<int> &state) {
  goal_counter_ = r_problem_->n_goal_facts();
  std::fill(best_support_.begin(), best_support_.end(), -1);
  std::fill(prop_cost_.begin(), prop_cost_.end(), -1);
  std::fill(is_applicable_.begin(), is_applicable_.end(), false);
  q_->Clear();

  if (more_helpful_)
    for (int i = 0, n = r_problem_->n_facts(); i < n; ++i)
      supporters_[i].clear();

  for (int i = 0, n = r_problem_->n_actions(); i < n; ++i) {
    precondition_counter_[i] = r_problem_->PreconditionSize(i);
    op_cost_[i] = r_problem_->ActionCost(i);
  }
}

void RPGTable::MayPush(int f, int a) {
  int op_c = op_cost_[a];

  if (op_c < prop_cost_[f] || prop_cost_[f] == -1) {
    if (more_helpful_) supporters_[f].clear();

    best_support_[f] = a;
    prop_cost_[f] = op_c;
    q_->Push(op_c, f);
  }

  if (more_helpful_ && op_c == prop_cost_[f]) supporters_[f].push_back(a);
}

void RPGTable::ConstructRRPG(const vector<int> &state,
                             const vector<bool> &black_list) {
  SetUp(state);

  for (int i = 0, n = r_problem_->n_actions(); i < n; ++i) {
    if (precondition_counter_[i] == 0) {
      is_applicable_[r_problem_->ActionId(i)] = true;
      if (!black_list[i]) MayPush(r_problem_->Effect(i), i);
    }
  }

  for (auto f : state) {
    prop_cost_[f] = 0;
    q_->Push(0, f);
  }

  while (!q_->IsEmpty()) {
    int c = q_->MinimumValue();
    int f = q_->Pop();

    assert(prop_cost_[f] != -1);

    if (prop_cost_[f] < c) continue;

    for (auto a : r_problem_->PreconditionMap(f)) {
      op_cost_[a] = std::max(r_problem_->ActionCost(a) + c, op_cost_[a]);

      if (--precondition_counter_[a] == 0) {
        is_applicable_[r_problem_->ActionId(a)] = true;
        if (!black_list[a]) MayPush(r_problem_->Effect(a), a);
      }
    }
  }
}

void RPGTable::DisjunctiveHelpful(const vector<int> &state,
                                  const vector<int> &disjunctive_goals,
                                  unordered_set<int> &helpful) {
  SetUp(state);
  std::fill(is_disjunctive_goal_.begin(), is_disjunctive_goal_.end(), false);

  for (auto g : disjunctive_goals) is_disjunctive_goal_[g] = true;

  int f = DisjunctiveGeneralizedDijkstra(state);

  if (f == -1) return;

  std::fill(plan_set_.begin(), plan_set_.end(), false);
  std::fill(marked_.begin(), marked_.end(), false);
  helpful.clear();
  SetPlan(f, helpful);
}

int RPGTable::DisjunctiveGeneralizedDijkstra(const vector<int> &state) {
  for (int i = 0, n = r_problem_->n_actions(); i < n; ++i) {
    if (precondition_counter_[i] == 0) {
      is_applicable_[r_problem_->ActionId(i)] = true;
      MayPush(r_problem_->Effect(i), i);
    }
  }

  for (auto f : state) {
    prop_cost_[f] = 0;
    q_->Push(0, f);
  }

  while (!q_->IsEmpty()) {
    int c = q_->MinimumValue();
    auto f = q_->Pop();

    assert(prop_cost_[f] != -1);

    if (prop_cost_[f] < c) continue;
    if (is_disjunctive_goal_[f]) return f;

    for (auto a : r_problem_->PreconditionMap(f)) {
      op_cost_[a] += c;

      if (--precondition_counter_[a] == 0) {
        is_applicable_[r_problem_->ActionId(a)] = true;
        MayPush(r_problem_->Effect(a), a);
      }
    }
  }

  return -1;
}

}  // namespace pplanner
