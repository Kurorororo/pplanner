#ifndef EFFECTS_H_
#define EFFECTS_H_

#include <vector>

#include "problem/partial_state_vector.h"

namespace pplanner {

class EffectVector : public PartialStateVector {
 public:
  EffectVector() : PartialStateVector() {}

  explicit EffectVector(size_t size) : PartialStateVector(size) {}

  void Apply(int i, std::vector<int> &state) const;
};

}

#endif // EFFECTS_H_
