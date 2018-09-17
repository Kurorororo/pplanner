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

  void Apply(int i, std::vector<int> &state) const;

  void AddConditionalEffect(
      const std::vector<std::vector<std::pair<int, int> > > &conditions,
      const std::vector<std::pair<int, int> > &effects);

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
