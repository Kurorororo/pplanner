#include "multithread_search/high_water_mark_compare.h"

namespace pplanner {

bool HighWaterMarkCompare::operator()(const std::set<int> &a,
                                      const std::set<int> &b) {
  auto a_itr = a.begin();
  auto a_end = a.end();
  auto b_itr = b.begin();
  auto b_end = b.end();

  while (a_itr != a_end && b_itr != b_end) {
    if (*a_itr < *b_itr) return true;

    if (*a_itr > *b_itr) return false;

    if (a_itr == b_itr) {
      ++a_itr;
      ++b_itr;
    }
  }

  return false;
}

}  // namespace pplanner