#include "problem/partial_state.h"

namespace pplanner {

bool PartialState::IsSubset(const std::vector<int> &state) const {
  for (int i=0, n=static_cast<int>(size()); i<n; ++i)
    if (state[vars_[i]] != values_[i]) return false;

  return true;
}

} // namespace pplanner
