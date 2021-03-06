#include "landmark/landmark.h"

#include <algorithm>
#include <iostream>

#include <boost/functional/hash.hpp>

#include "hash/pair_hash.h"

namespace pplanner {

using std::pair;
using std::vector;

bool Landmark::IsImplicated(const vector<int> &state) const {
  for (auto p : var_values_)
    if (state[p.first] == p.second) return true;

  return false;
}

bool Landmark::IsImplicated(const pair<int, int> &p) const {
  for (auto v : var_values_)
    if (v == p) return true;

  return false;
}

bool Landmark::Overlap(const Landmark &l) const {
  if (*this == l) return false;

  for (auto v : var_values_)
    for (auto w : l.var_values_)
      if (v == w) return true;

  return false;
}

void Landmark::Dump() const {
  for (auto v : var_values_)
    std::cout << "var" << v.first << "->" << v.second << " ";
}

std::size_t Landmark::Hash() const {
  std::size_t seed = 0;

  for (auto v : var_values_)
    boost::hash_combine(seed, PairHash<int, int>{}(v));

  return seed;
}

} // namespace pplanner
