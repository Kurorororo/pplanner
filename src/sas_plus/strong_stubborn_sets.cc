#include "strong_stubborn_sets.h"

#include <iostream>

namespace pplanner {

using std::vector;

void SSSApproximater::ApproximateSSS(const vector<int> &state,
                                     const vector<int> &applicable,
                                     vector<bool> &sss) {
  static vector<int> new_comers_0;
  static vector<int> new_comers_1;

  sss.resize(problem_->n_actions());
  std::fill(sss.begin(), sss.end(), false);
  new_comers_0.clear();
  new_comers_1.clear();

  int var = -1;
  int value = -1;
  FDOrderGoal(state, &var, &value);
  if (var == -1 && value == -1) return;
  int f = problem_->Fact(var, value);

  int size = 0;

  for (auto a : to_achievers_[f]) {
    sss[a] = true;
    new_comers_0.push_back(a);
    ++size;
  }

  while (true) {
    int before_size = size;

    for (auto a : new_comers_0) {
      FDOrderPrecondition(state, a, &var, &value);

      if (var == -1 && value == -1) {
        if (!interfere_computed_[a]) ComputeInterfere(a);

        for (auto b : to_interfere_[a]) {
          if (sss[b]) continue;
          sss[b] = true;
          new_comers_1.push_back(b);
          ++size;
        }
      } else {
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

void SSSApproximater::ComputeInterfere(int a) {
  for (int b=0, n=problem_->n_actions(); b<n; ++b)
    if (AreInterfere(a, b))
      to_interfere_[a].push_back(b);

  interfere_computed_[a] = true;
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

bool SSSApproximater::PreconditionsMutex(int a, int b) const {
  auto a_var_itr = problem_->PreconditionVarsBegin(a);
  auto a_var_end = problem_->PreconditionVarsEnd(a);
  auto a_value_itr = problem_->PreconditionValuesBegin(a);

  auto b_var_itr = problem_->PreconditionVarsBegin(b);
  auto b_var_end = problem_->PreconditionVarsEnd(b);
  auto b_value_itr = problem_->PreconditionValuesBegin(b);

  while (a_var_itr != a_var_end && b_var_itr != b_var_end) {
    int a_var = *a_var_itr;
    int a_value = *a_value_itr;

    int b_var = *b_var_itr;
    int b_value = *b_value_itr;

    if ((b_var == a_var && b_value != a_value)
       || problem_->IsMutex(a_var, a_value, b_var, b_value))
      return true;

    if (a_var < b_var) {
      ++a_var_itr;
      ++a_value_itr;
    } else if (b_var < a_var) {
      ++b_var_itr;
      ++b_value_itr;
    } else {
      ++a_var_itr;
      ++a_value_itr;
      ++b_var_itr;
      ++b_value_itr;
    }
  }

  return false;
}

bool SSSApproximater::Disable(int a, int b) const {
  auto a_var_itr = problem_->EffectVarsBegin(a);
  auto a_var_end = problem_->EffectVarsEnd(a);
  auto a_value_itr = problem_->EffectValuesBegin(a);

  auto b_var_itr = problem_->PreconditionVarsBegin(b);
  auto b_var_end = problem_->PreconditionVarsEnd(b);
  auto b_value_itr = problem_->PreconditionValuesBegin(b);

  while (a_var_itr != a_var_end && b_var_itr != b_var_end) {
    if (*b_var_itr == *a_var_itr && *b_value_itr != *a_value_itr)
      return true;

    if (*a_var_itr < *b_var_itr) {
      ++a_var_itr;
      ++a_value_itr;
    } else if (*b_var_itr < *a_var_itr) {
      ++b_var_itr;
      ++b_value_itr;
    } else {
      ++a_var_itr;
      ++a_value_itr;
      ++b_var_itr;
      ++b_value_itr;
    }
  }

  return false;
}

bool SSSApproximater::Conflict(int a, int b) const {
  auto a_var_itr = problem_->EffectVarsBegin(a);
  auto a_var_end = problem_->EffectVarsEnd(a);
  auto a_value_itr = problem_->EffectValuesBegin(a);

  auto b_var_itr = problem_->EffectVarsBegin(b);
  auto b_var_end = problem_->EffectVarsEnd(b);
  auto b_value_itr = problem_->EffectValuesBegin(b);

  while (a_var_itr != a_var_end && b_var_itr != b_var_end) {
    if (*b_var_itr == *a_var_itr && *b_value_itr != *a_value_itr)
      return true;

    if (*a_var_itr < *b_var_itr) {
      ++a_var_itr;
      ++a_value_itr;
    } else if (*b_var_itr < *a_var_itr) {
      ++b_var_itr;
      ++b_value_itr;
    } else {
      ++a_var_itr;
      ++a_value_itr;
      ++b_var_itr;
      ++b_value_itr;
    }
  }

  return false;
}

} // namespace pplanner
