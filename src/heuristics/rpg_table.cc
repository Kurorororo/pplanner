#include "rpg_table.h"

namespace pplanner {

using std::vector;

int RPGTable::PlanCost(const vector<int> &state) {
  int additive_h = AdditiveCost(state);
  if (additive_h == -1) return -1;

  for (auto g : problem_->goal())
    SetPlan(g);

  std::fill(marked_.begin(), marked_.end(), false);
  std::fill(plan_set_.begin(), plan_set_.end(), false);

  int h = 0;

  for (int i=0, n=plan_set_.size(); i<n; ++i)
    if (plan_set_[i]) h += problem_->ActionCosts(i);

  return h;
}

int RPGTable::AdditiveCost(const vector<int> &state) {
  GeneralizedDijkstra(state);

  int h = 0;

  for (auto g : problem_->goal()) {
    int cost = prop_cost_[g];
    if (cost == -1) return cost;
    h += cost;
  }

  return h;
}

void RPGTable::SetPlan(int g) {
  if (marked_[g]) return;

  marked_[g] = true;
  int unary_a = best_support_[g];
  if (unary_a == -1) return;

  for (auto p : problem_->Preconditions(unary_a))
    SetPlan(p);

  int a = ids_[unary_a];
  plan_set_[a] = true;
}

void RPGTable::GeneralizedDijkstra(const vector<int> &state) {
  SetUp(state);

  while (!q_.empty()) {
    auto top = q_.top();
    q_.pop();

    int c = top.first;
    int f = top.second;

    assert(prop_cost_[f] != -1);

    if (prop_cost_[f] < c) continue;
    if (is_goal_[f] && --goal_counter_ == 0) return;

    for (auto a : problem_->PreconditionMap(f)) {
      op_cost_[a] += c;

      if (--precondition_counter_[a] == 0)
        MayPush(problem_->Effect(a), a);
    }
  }
}

void RPGTable::SetUp(const vector<int> &state) {
  table->goal_counter_ = problem_->n_goal_facts();
  std::fill(best_support_.begin(), best_support_.end(), -1);
  std::fill(prop_cost_.begin(), prop_cost_.end(), -1);
  q_ = PQueue();

  for (int i=0, n=problem_->n_actions(); i<n; ++i) {
    precondition_counter_[i] = problem_->PreconditionSize(i);
    op_cost_[i] = problem_->ActionCosts(i);

    if (precondition_counter_[i] == 0)
      MayPush(problem_->Effects(i), i);
  }

  for (auto f : state) {
    prop_cost_[f] = 0;
    q.push(std::make_pair(0, f));
  }
}

void RPGTable::MayPush(int f, int a) {
  int op_c = op_cost_[a];

  if (op_c < prop_cost_[f] || prop_cost_[f] == -1) {
    best_support_[f] = a;
    prop_cost_[f] = op_c;
    q_.push(std::make_pair(op_c, f));
  }
}

} // namespace pplanner
