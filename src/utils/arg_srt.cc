#include "arg_sort.h"

#include <algorithm>

using std::vector;

namespace pplanner {

void ArgSort(const vector<int> &v, vector<int> &indices) {
  indices.resize(v.size());
  std::iota(indices.begin(), indices.end(), 0);
  auto compare = [const &v](int l, int r) { return v[l] < v[r]; };
  std::sort(indices.begin(), indices.end(), compare);
}

} // namespace pplanner
