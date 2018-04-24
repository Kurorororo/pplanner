#include "problem/effects.h"

namespace pplanner {

void EffectVector::Apply(int i, std::vector<int> &state) {
  auto value_iter = values_begin(i);

  for (auto iter = vars_begin(i); iter != vars_end(i); ++iter) {
    state[*iter] = *value_iter;
    ++value_iter;
  }
}

};
