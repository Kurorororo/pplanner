#include "sas_plus/partial_state_vector.h"

#include <iostream>

using std::pair;
using std::vector;

namespace pplanner {

void PartialStateVector::Copy(int i, vector<pair<int, int> > &v) const {
  int size = SizeOfPartialState(i);
  v.resize(size);

  auto var_iter = VarsBegin(i);
  auto value_iter = ValuesBegin(i);

  for (auto &p : v) {
    p.first = *var_iter;
    p.second = *value_iter;
    ++var_iter;
    ++value_iter;
  }
}

void PartialStateVector::Add(const vector<pair<int, int> > &v) {
  int size = v.size();

  for (auto &f : v) {
    vars_.push_back(f.first);
    values_.push_back(f.second);
  }

  offsets_.push_back(offsets_.back() + static_cast<int>(size));
}

void PartialStateVector::Dump(int i) const {
  auto var_iter = VarsBegin(i);
  auto value_iter = ValuesBegin(i);
  int n = static_cast<int>(SizeOfPartialState(i));
  std::cout << n << " assignments" << std::endl;

  for (int j=0; j<n; ++j) {
    std::cout << "var" << *var_iter << "=" << *value_iter << ", ";
    ++var_iter;
    ++value_iter;
  }

  std::cout << std::endl;
}

} // namespace pplanner
