#ifndef ARG_SORT_H_
#define ARG_SORT_H_

#include <algorithm>
#include <numeric>
#include <vector>

namespace pplanner {

template<typename T>
void ArgSort(const std::vector<T> &v, std::vector<int> &indices,
             bool greater=false) {
  indices.resize(v.size());
  std::iota(indices.begin(), indices.end(), 0);

  if (greater) {
    auto compare = [&v](int l, int r) { return v[l] > v[r]; };
    std::sort(indices.begin(), indices.end(), compare);
  } else {
    auto compare = [&v](int l, int r) { return v[l] < v[r]; };
    std::sort(indices.begin(), indices.end(), compare);
  }
}

template<typename T>
std::vector<int> ArgSort(const std::vector<T> &v, bool greater=false) {
  std::vector<int> indices;
  ArgSort<T>(v, indices, greater);

  return indices;
}

} // namespace pplanner

#endif // ARG_SORT_H_
