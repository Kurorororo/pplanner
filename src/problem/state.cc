#include "domain/state.h"

#include <cassert>

namespace rwls {

void ApplyEffect(const std::vector<VarValue> &effect, State &variables) {
  for (auto v : effect) {
    int var, value;
    DecodeVarValue(v, &var, &value);
    variables[var] = value;
  }
}

bool GoalCheck(const std::vector<VarValue> &goal, const State &variables) {
  for (auto v : goal) {
    int var, value;
    DecodeVarValue(v, &var, &value);
    if (variables[var] != value) return false;
  }
  return true;
}

} // namespace rwls
