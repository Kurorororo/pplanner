#include "sas_plus/partial_state.h"

#include <iostream>

using std::pair;
using std::vector;

namespace pplanner {

PartialState::PartialState(const vector<pair<int, int> > &v) {
  int size = v.size();
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

void PartialState::Copy(vector<pair<int, int> > &v) const {
  v.clear();

  for (int i=0, n=size(); i<n; ++i)
    v.push_back(std::make_pair(vars_[i], values_[i]));
}

void PartialState::Dump() const {
  int n = static_cast<int>(size());
  std::cout << n << " assignments" << std::endl;

  for (int i=0; i<n; ++i)
    std::cout << "var" << vars_[i] << "=" << values_[i] << ", ";

  std::cout << std::endl;
}

} // namespace pplanner
