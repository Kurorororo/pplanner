#include "landmark/landmark.h"

#include <algorithm>
#include <iostream>

#include <boost/functional/hash.hpp>

namespace rwls {

bool Landmark::IsImplicated(const State &state) const {
  for (auto v : var_values_) {
    int var, value;
    DecodeVarValue(v, &var, &value);
    if (state[var] == value) return true;
  }
  return false;
}

bool Landmark::IsImplicated(const std::vector<VarValue> &assignment) const {
  for (auto v : assignment) {
    auto result = std::find(var_values_.begin(), var_values_.end(), v);
    if (result != var_values_.end()) return true;
  }
  return false;
}

bool Landmark::IsImplicated(int var, int value) const {
  for (auto v : var_values_) {
    int v_var, v_value;
    DecodeVarValue(v, &v_var, &v_value);
    if (v_var == var && v_value == value) return true;
  }
  return false;
}

void Landmark::Print() const {
  for (auto v : var_values_) {
    int var, value;
    DecodeVarValue(v, &var, &value);
    std::cout << "var" << var << "->" << value << " ";
  }
}

size_t Landmark::Hash() const {
  size_t seed = 0;
  for (auto v : var_values_)
    boost::hash_combine(seed, v);
  return seed;
}

}
