#include "sas_plus/effect_vector.h"

namespace pplanner {

void EffectVector::Apply(int i, std::vector<int> &state) const {
  auto value_iter = ValuesBegin(i);

  for (auto iter = VarsBegin(i); iter != VarsEnd(i); ++iter) {
    state[*iter] = *value_iter;
    ++value_iter;
  }
}

};
