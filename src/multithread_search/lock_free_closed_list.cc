#include "multithread_search/lock_free_closed_list.h"

namespace pplanner {

void LockFreeClosedList::Init() {
  for (int i = 0, n = closed_.size(); i < n; ++i)
    closed_[i].store(nullptr);
}

bool LockFreeClosedList::IsClosed(
    uint32_t hash,
    const std::vector<uint32_t> &packed_state) const {
  std::size_t i = hash & mask_;
  auto node = closed_[i].load();

  while (node != nullptr) {
    if (packed_state == node->packed_state) return true;
    node = node->next.load();
  }

  return false;
}

void LockFreeClosedList::Close(SearchNodeWithNext *node) {
  std::size_t i = node->hash & mask_;
  auto expected = closed_[i].load();

  while (!closed_[i].compare_exchange_weak(expected, node));

  node->next.store(expected);
}

} // namespace pplanner
