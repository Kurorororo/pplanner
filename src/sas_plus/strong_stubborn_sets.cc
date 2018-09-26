#include "strong_stubborn_sets.h"

#include <iostream>

namespace pplanner {

using std::vector;

void SSSApproximater::ApproximateSSS(const vector<int> &state,
                                     const vector<int> &applicable,
                                     vector<bool> &sss) const {
  static vector<bool> applicable_set(problem_->n_actions(), false);
  static vector<int> new_comers_0;
  static vector<int> new_comers_1;

  sss.resize(problem_->n_actions());
  std::fill(sss.begin(), sss.end(), false);
  std::fill(applicable_set.begin(), applicable_set.end(), false);

  for (auto a : applicable)
    applicable_set[a] = true;

  int var = -1;
  int value = -1;
  FDOrderGoal(state, &var, &value);
  if (var == -1 && value == -1) return;
  int f = problem_->Fact(var, value);

  new_comers_0.clear();
  new_comers_1.clear();
  int size = 0;

  for (auto a : to_achievers_[f]) {
    sss[a] = true;
    new_comers_0.push_back(a);
    ++size;
  }

  while (true) {
    int before_size = size;

    for (auto a : new_comers_0) {
      if (applicable_set[a]) {
        for (int b=0, n=problem_->n_actions(); b<n; ++b) {
        //for (auto b : to_interfere_[a]) {
          if (sss[b]) continue;
          if (!MutexInterfere(a, b)) continue;
          sss[b] = true;
          new_comers_1.push_back(b);
          ++size;
        }
      } else {
        FDOrderPrecondition(state, a, &var, &value);
        int f = problem_->Fact(var, value);

        for (auto b : to_achievers_[f]) {
          if (sss[b]) continue;
          sss[b] = true;
          new_comers_1.push_back(b);
          ++size;
        }
      }
    }

    if (size == before_size) break;

    new_comers_0.swap(new_comers_1);
    new_comers_1.clear();
  }
}

void SSSApproximater::InitAchievers() {
  to_achievers_.resize(problem_->n_facts(), vector<int>());

  for (int i=0, n=problem_->n_actions(); i<n; ++i) {
    auto var_itr = problem_->EffectVarsBegin(i);
    auto var_end = problem_->EffectVarsEnd(i);
    auto value_itr = problem_->EffectValuesBegin(i);

    while (var_itr != var_end) {
      int f = problem_->Fact(*var_itr, *value_itr);
      to_achievers_[f].push_back(i);

      ++var_itr;
      ++value_itr;
    }
  }
}

void SSSApproximater::InitInterfere() {
  int n = problem_->n_actions();
  to_interfere_.resize(n, vector<int>());

  for (int i=0; i<n; ++i) {
    for (int j=0; j<n; ++j) {
      if (i == j) continue;

      if (MutexInterfere(i, j)) to_interfere_[i].push_back(j);
    }
  }
}

void SSSApproximater::FDOrderGoal(const vector<int> &state, int *goal_var,
                                 int *goal_value) const {
  *goal_var = -1;
  *goal_value = -1;

  for (int i=0, n=problem_->n_goal_facts(); i<n; ++i) {
    int var = problem_->GoalVar(i);
    int value = problem_->GoalValue(i);

    if (state[var] != value) {
      *goal_var = var;
      *goal_value = value;

      return;
    }
  }
}

void SSSApproximater::FDOrderPrecondition(const vector<int> &state, int a,
                                         int *goal_var, int *goal_value) const {
  *goal_var = -1;
  *goal_value = -1;

  auto var_itr = problem_->PreconditionVarsBegin(a);
  auto var_end = problem_->PreconditionVarsEnd(a);
  auto value_itr = problem_->PreconditionValuesBegin(a);

  while (var_itr != var_end) {
    int var = *var_itr;
    int value = *value_itr;

    if (state[var] != value) {
      *goal_var = var;
      *goal_value = value;

      return;
    }

    ++var_itr;
    ++value_itr;
  }
}

bool SSSApproximater::MutexInterfere(int a, int b) const {
  auto a_var_itr = problem_->PreconditionVarsBegin(a);
  auto a_var_end = problem_->PreconditionVarsEnd(a);
  auto a_value_itr = problem_->PreconditionValuesBegin(a);

  while (a_var_itr != a_var_end) {
    int a_var = *a_var_itr;
    int a_value = *a_value_itr;

    auto b_var_itr = problem_->PreconditionVarsBegin(b);
    auto b_var_end = problem_->PreconditionVarsEnd(b);
    auto b_value_itr = problem_->PreconditionValuesBegin(b);

    while (b_var_itr != b_var_end) {
      int b_var = *b_var_itr;
      int b_value = *b_value_itr;

      if ((a_var == b_var && a_value != b_value)
          || problem_->IsMutex(a_var, a_value, b_var, b_value)) {
        return false;
      }

      ++b_var_itr;
      ++b_value_itr;
    }

    ++a_var_itr;
    ++a_value_itr;
  }

  return true;
}

} // namespace pplanner
