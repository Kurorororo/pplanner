#ifndef PAIR_HASH_H_
#define PAIR_HASH_H_

#include <functional>

#include <cstddef>

namespace pplanner {

template<typename T, typename U>
struct PairHash {
  std::size_t operator()(const std::pair<T, U> &p) const {
    return std::hash<T>{}(p.first) ^ std::hash<U>{}(p.second);
  }
};

} // namespace pplanner

#endif // PAIR_HASH_H_
