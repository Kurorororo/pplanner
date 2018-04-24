#ifndef STATE_H_
#define STATE_H_

#include <vector>

#include <boost/functional/hash.hpp>

#include "domain/var_value.h"

namespace rwls {

using State = std::vector<int>;

struct StateHash {
  inline size_t operator()(const State &state) const {
    return boost::hash_range(state.begin(), state.end());
  }
};

void ApplyEffect(const std::vector<VarValue> &effect, State &variables);

bool GoalCheck(const std::vector<VarValue> &goal, const State &variables);

}

#endif
