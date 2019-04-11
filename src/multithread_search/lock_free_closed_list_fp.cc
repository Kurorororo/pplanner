#include "multithread_search/lock_free_closed_list_fp.h"

namespace pplanner {

/***
 * Compare and swap the root node of each linked list.
 * Initially, root nodes are nullptr.
 * Entries in closed lists are deleted after the entire search,
 * so an ABA problem would not happen.
 */

void LockFreeClosedListFP::Init() {
  for (int i = 0, n = closed_.size(); i < n; ++i) closed_[i].store(nullptr);
}

bool LockFreeClosedListFP::IsClosed(
    uint32_t hash, const std::vector<uint32_t> &packed_state) const {
  std::size_t i = hash & mask_;
  auto node = closed_[i].load();

  while (node != nullptr) {
    if (packed_state == node->packed_state) return true;
    node = node->next.load();
  }

  return false;
}

void LockFreeClosedListFP::Close(SearchNodeWithNext *node) {
  std::size_t i = node->hash & mask_;
  auto expected = closed_[i].load();

  while (!closed_[i].compare_exchange_weak(expected, node))
    ;

  node->next.store(expected);
}

}  // namespace pplanner
