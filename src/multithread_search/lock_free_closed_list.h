#ifndef LOCK_FREE_CLOSED_LIST_H_
#define LOCK_FREE_CLOSED_LIST_H_

#include <atomic>
#include <unordered_set>
#include <vector>

#include "multithread_search/search_node.h"

namespace pplanner {

struct SearchNodeWithNext : public SearchNode {
  std::atomic<SearchNodeWithNext*> next;
};

class LockFreeClosedList {
 public:
  LockFreeClosedList(int exponent=26)
    : mask_((1u << exponent) - 1), closed_(1 << exponent) {
    Init();
  }

  bool IsClosed(uint32_t hash, const std::vector<uint32_t> &packed_state) const;

  void Close(SearchNodeWithNext* node);

 private:
  void Init();

  uint32_t mask_;
  std::vector<std::atomic<SearchNodeWithNext*> > closed_;
};


} // namespace pplanner

#endif // LOCK_FREE_CLOSED_LIST_H_
