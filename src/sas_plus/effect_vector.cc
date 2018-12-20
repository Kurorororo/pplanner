#include "sas_plus/effect_vector.h"

#include <iostream>

namespace pplanner {

using std::vector;
using std::pair;

void EffectVector::Apply(int i, vector<int> &state) const {
  thread_local vector<int> backup(state);

  if (use_conditional_ && has_conditional_[i]) {
    backup = state;
    int begin = effect_condition_offsets_1_[i];
    int end = effect_condition_offsets_1_[i + 1];

    for (int j = begin; j < end; ++j) {
      int b = effect_condition_offsets_2_[j];
      int e = effect_condition_offsets_2_[j + 1];
      bool all = true;

      for (int k = b; k < e; ++k) {
        int var = effect_condition_vars_[k];
        int value = effect_condition_values_[k];

        if (backup[var] != value) {
          all = false;
          break;
        }
      }

      if (all)
        state[conditional_effect_vars_[j]] = conditional_effect_values_[j];
    }
  }

  auto value_iter = ValuesBegin(i);

  for (auto iter = VarsBegin(i); iter != VarsEnd(i); ++iter) {
    state[*iter] = *value_iter;
    ++value_iter;
  }
}

void EffectVector::AddConditionalEffect(
    const vector<vector<pair<int, int> > > &conditions,
    const vector<pair<int, int> > &effects) {

  if (conditions.empty()) {
    has_conditional_.push_back(false);
    effect_condition_offsets_1_.push_back(0);

    return;
  }

  if (!use_conditional_) use_conditional_ = true;

  has_conditional_.push_back(true);
  int offset_1 = effect_condition_offsets_1_.back() + conditions.size();
  effect_condition_offsets_1_.push_back(offset_1);

  for (auto &condition : conditions) {
    for (auto p : condition) {
      effect_condition_vars_.push_back(p.first);
      effect_condition_values_.push_back(p.second);
    }

    int offset_2 = effect_condition_offsets_2_.back() + condition.size();
    effect_condition_offsets_2_.push_back(offset_2);
  }

  for (auto p : effects) {
    conditional_effect_vars_.push_back(p.first);
    conditional_effect_values_.push_back(p.second);
  }
}

void EffectVector::CopyEffectConditions(
    int i,
    vector<vector<pair<int, int> > > &conditions) const {
  conditions.clear();
  int begin = effect_condition_offsets_1_[i];
  int end = effect_condition_offsets_1_[i + 1];

  for (int j = begin; j < end; ++j) {
    int b = effect_condition_offsets_2_[j];
    int e = effect_condition_offsets_2_[j + 1];
    conditions.push_back(vector<pair<int, int> >());

    for (int k = b; k < e; ++k) {
      int var = effect_condition_vars_[k];
      int value = effect_condition_values_[k];
      conditions.back().push_back(std::make_pair(var, value));
    }
  }
}

void EffectVector::CopyConditionalEffects(
    int i,
    vector<pair<int, int> > &effects) const {
  effects.clear();
  int begin = effect_condition_offsets_1_[i];
  int end = effect_condition_offsets_1_[i + 1];

  for (int j = begin; j < end; ++j) {
    int var = conditional_effect_vars_[j];
    int value = conditional_effect_values_[j];
    effects.push_back(std::make_pair(var, value));
  }
}

void EffectVector::Dump(int i) const {
  PartialStateVector::Dump(i);

  if (use_conditional_ && has_conditional_[i]) {
    int begin = effect_condition_offsets_1_[i];
    int end = effect_condition_offsets_1_[i + 1];
    std::cout << end - begin << " conditional effects" << std::endl;

    for (int j = begin; j < end; ++j) {
      int b = effect_condition_offsets_2_[j];
      int e = effect_condition_offsets_2_[j + 1];
      std::cout << e - b << " assignments" << std::endl;

      for (int k = b; k < e; ++k) {
        int var = effect_condition_vars_[k];
        int value = effect_condition_values_[k];

        std::cout << "var" << var << "=" << value << ", ";
      }

      std::cout << std::endl << "effect: " << "var";
      std::cout << conditional_effect_vars_[j] << "=";
      std::cout << conditional_effect_values_[j] << std::endl;;
    }
  }
}

};
