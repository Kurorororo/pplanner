#include "sas_plus/partial_state.h"

using std::pair;
using std::vector;

namespace pplanner {

PartialState::PartialState(const vector<pair<int, int> > &v) {
  size_t size = v.size();
  vars_.reserve(size);
  values_.reserve(size);

  for (auto f : v) {
    vars_.push_back(f.first);
    values_.push_back(f.second);
  }
}

bool PartialState::IsSubset(const vector<int> &state) const {
  for (int i=0, n=static_cast<int>(size()); i<n; ++i)
    if (state[vars_[i]] != values_[i]) return false;

  return true;
}

} // namespace pplanner
