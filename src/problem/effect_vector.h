#ifndef EFFECT_VECTOR_H_
#define EFFECT_VECTOR_H_

#include <vector>

#include "problem/partial_state_vector.h"

namespace pplanner {

class EffectVector : public PartialStateVector {
 public:
  EffectVector() : PartialStateVector() {}

  void Apply(int i, std::vector<int> &state) const;
};

}

#endif // EFFECT_VECTOR_H_
