#ifndef EFFECT_VECTOR_H_
#define EFFECT_VECTOR_H_

#include <vector>

#include "sas_plus/partial_state_vector.h"

namespace pplanner {

class EffectVector : public PartialStateVector {
 public:
  EffectVector() : PartialStateVector(), use_conditional_(false),
                   effect_condition_offsets_1_(1, 0),
                   effect_condition_offsets_2_(1, 0) {}

  void Dump(int i) const override;

  void Apply(int i, const std::vector<int> &state, std::vector<int> &child)
    const;

  void AddConditionalEffect(
      const std::vector<std::vector<std::pair<int, int> > > &conditions,
      const std::vector<std::pair<int, int> > &effects);

  bool HasConditionalEffects(int i) const { return has_conditional_[i]; }

  int NConditionalEffects(int i) const {
    return effect_condition_offsets_1_[i + 1] - effect_condition_offsets_1_[i];
  }

  bool Condition(int i, int j, const std::vector<int> &state) const;

  bool NCondition(int i, int j) const {
    int b = effect_condition_offsets_2_[effect_condition_offsets_1_[i] + j];
    int e = effect_condition_offsets_2_[effect_condition_offsets_1_[i] + j + 1];

    return e - b;
  }

  bool ConditionVar(int i, int j, int k) const {
    int b = effect_condition_offsets_2_[effect_condition_offsets_1_[i] + j];

    return effect_condition_vars_[b + k];
  }

  bool ConditionValue(int i, int j, int k) const {
    int b = effect_condition_offsets_2_[effect_condition_offsets_1_[i] + j];

    return effect_condition_values_[b + k];
  }

  int ConditionalEffectVar(int i, int j) const {
    return conditional_effect_vars_[effect_condition_offsets_1_[i] + j];
  }

  int ConditionalEffectValue(int i, int j) const {
    return conditional_effect_values_[effect_condition_offsets_1_[i] + j];
  }

  void ApplyConditionalEffect(int i, int j, std::vector<int> &state) const {
    state[ConditionalEffectVar(i, j)] = ConditionalEffectValue(i, j);
  }

  void CopyEffectConditions(
      int i,
      std::vector<std::vector<std::pair<int, int> > > &conditions) const;

  void CopyConditionalEffects(int i,
                              std::vector<std::pair<int, int> > &effects) const;

  bool use_conditional() const { return use_conditional_; }

  const int* effect_condition_offsets_1_data() const {
    return effect_condition_offsets_1_.data();
  }

  std::size_t effect_condition_offsets_2_size() const {
    return effect_condition_offsets_2_.size();
  }

  const int* effect_condition_offsets_2_data() const {
    return effect_condition_offsets_2_.data();
  }

  std::size_t effect_conditions_size() const {
    return effect_condition_vars_.size();
  }

  const int* effect_condition_vars_data() const {
    return effect_condition_vars_.data();
  }

  const int* effect_condition_values_data() const {
    return effect_condition_values_.data();
  }

  std::size_t conditional_effects_size() const {
    return conditional_effect_vars_.size();
  }

  const int* conditional_effect_vars_data() const {
    return conditional_effect_vars_.data();
  }

  const int* conditional_effect_values_data() const {
    return conditional_effect_values_.data();
  }

 private:
  bool use_conditional_;
  std::vector<bool> has_conditional_;
  std::vector<int> effect_condition_offsets_1_;
  std::vector<int> effect_condition_offsets_2_;
  std::vector<int> effect_condition_vars_;
  std::vector<int> effect_condition_values_;
  std::vector<int> conditional_effect_vars_;
  std::vector<int> conditional_effect_values_;
};

}

#endif // EFFECT_VECTOR_H_
