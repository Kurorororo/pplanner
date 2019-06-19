#ifndef HIGH_WATER_MARK_COMPARE_H_
#define HIGH_WATER_MARK_COMPARE_H_

#include <set>

namespace pplanner {

struct HighWaterMarkComapare {
  bool operator()(const std::set<int> &a, const std::set<int> &b);
};

}  // namespace pplanner

#endif  // HIGH_WATER_MARK_COMPARE_H_