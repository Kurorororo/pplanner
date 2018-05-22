#include "rpg_table.h"

namespace pplanner {

using std::unordered_set;
using std::vector;

int RPGTable::PlanCost(const vector<int> &state, unordered_set<int> &helpful,
                       bool unit_cost, bool more_helpful) {
  int additive_h = AdditiveCost(state, unit_cost, more_helpful);
  if (additive_h == -1) return -1;

  std::fill(plan_set_.begin(), plan_set_.end(), false);
  std::fill(marked_.begin(), marked_.end(), false);
  helpful.clear();

  for (auto g : r_problem_->goal())
    SetPlan(g, helpful, more_helpful);

  int h = 0;

  for (int i=0, n=plan_set_.size(); i<n; ++i) {
    if (plan_set_[i]) {
      if (unit_cost)
        ++h;
      else
        h += problem_->ActionCost(i);
    }
  }

  return h;
}

int RPGTable::AdditiveCost(const vector<int> &state, bool unit_cost,
                           bool more_helpful) {
  SetUp(state, unit_cost, more_helpful);
  GeneralizedDijkstra(state, more_helpful);

  int h = 0;

  for (auto g : r_problem_->goal()) {
    int cost = prop_cost_[g];
    if (cost == -1) return cost;
    h += cost;
  }

  return h;
}

void RPGTable::SetPlan(int g, unordered_set<int> &helpful, bool more_helpful) {
  if (marked_[g]) return;

  marked_[g] = true;
  int unary_a = best_support_[g];
  if (unary_a == -1) return;
  int a = r_problem_->ActionId(unary_a);
  plan_set_[a] = true;

  if (prop_cost_[g] == r_problem_->ActionCost(unary_a)) {
    helpful.insert(a);

    if (more_helpful)
      for (auto supporter : supporters_[g])
        helpful.insert(r_problem_->ActionId(supporter));
  }

  for (auto p : r_problem_->Precondition(unary_a))
    SetPlan(p, helpful, more_helpful);
}

void RPGTable::GeneralizedDijkstra(const vector<int> &state,
                                   bool more_helpful) {
  while (!q_.empty()) {
    auto top = q_.top();
    q_.pop();

    int c = top.first;
    int f = top.second;

    assert(prop_cost_[f] != -1);

    if (prop_cost_[f] < c) continue;
    if (r_problem_->IsGoal(f) && --goal_counter_ == 0) return;

    for (auto a : r_problem_->PreconditionMap(f)) {
      op_cost_[a] += c;

      if (--precondition_counter_[a] == 0)
        MayPush(r_problem_->Effect(a), a, more_helpful);
    }
  }
}

void RPGTable::SetUp(const vector<int> &state, bool unit_cost,
                     bool more_helpful) {
  goal_counter_ = r_problem_->n_goal_facts();
  std::fill(best_support_.begin(), best_support_.end(), -1);
  std::fill(prop_cost_.begin(), prop_cost_.end(), -1);
  q_ = PQueue();

  for (int i=0, n=r_problem_->n_facts(); i<n; ++i)
    supporters_[i].clear();

  for (int i=0, n=r_problem_->n_actions(); i<n; ++i) {
    precondition_counter_[i] = r_problem_->PreconditionSize(i);

    if (unit_cost)
      op_cost_[i] = 1;
    else
      op_cost_[i] = r_problem_->ActionCost(i);

    if (precondition_counter_[i] == 0)
      MayPush(r_problem_->Effect(i), i, more_helpful);
  }

  for (auto f : state) {
    prop_cost_[f] = 0;
    q_.push(std::make_pair(0, f));
  }
}

void RPGTable::MayPush(int f, int a, bool more_helpful) {
  int op_c = op_cost_[a];

  if (op_c < prop_cost_[f] || prop_cost_[f] == -1) {
    if (more_helpful) supporters_[f].clear();

    best_support_[f] = a;
    prop_cost_[f] = op_c;
    q_.push(std::make_pair(op_c, f));
  }

  if (more_helpful && op_c == prop_cost_[f])
    supporters_[f].push_back(a);
}


} // namespace pplanne
