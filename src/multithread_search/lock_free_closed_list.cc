#include "multithread_search/lock_free_closed_list.h"

namespace pplanner {

/***
 * Michael, M., 2002.
 * High Performance Dynamic Lock-Free Hash Tables and List-Based Sets
 * In state space search, there is no need for deletion of nodes in closed
 * lists, so we did not implement marking and deletion.
 * In addition, it is dirty that each mark uses the lowest bit of the pointer.
 */

void LockFreeClosedList::Init() {
  for (int i = 0, n = closed_.size(); i < n; ++i) closed_[i] = nullptr;
}

bool LockFreeClosedList::IsClosed(
    uint32_t hash, const std::vector<uint32_t> &packed_state) const {
  std::size_t i = hash & mask_;
  auto p = Find(i, packed_state);

  return p.first != nullptr;
}

bool LockFreeClosedList::Close(std::shared_ptr<SearchNodeWithNext> node) {
  std::size_t i = node->hash & mask_;

  while (true) {
    auto p = Find(i, node->packed_state);
    auto cur = p.first;
    auto prev = p.second;

    if (cur != nullptr && cur->packed_state == node->packed_state) return false;

    node->next = cur;

    if (prev == nullptr) {
      if (std::atomic_compare_exchange_weak(&closed_[i], &cur, node))
        return true;
    } else if (std::atomic_compare_exchange_weak(&prev->next, &cur, node)) {
      return true;
    }
  }
}

std::pair<std::shared_ptr<SearchNodeWithNext>,
          std::shared_ptr<SearchNodeWithNext> >
LockFreeClosedList::Find(std::size_t head_index,
                         const std::vector<uint32_t> &packed_state) const {
  while (true) {
    auto prev = closed_[head_index];

    if (prev == nullptr) return std::make_pair(nullptr, nullptr);

    auto cur = prev->next;

    if (closed_[head_index] != prev) continue;

    if (prev->packed_state >= packed_state)
      return std::make_pair(prev, nullptr);

    while (true) {
      if (cur == nullptr) return std::make_pair(nullptr, prev);

      auto next = cur->next;

      if (prev->next != cur) break;

      if (cur->packed_state >= packed_state) return std::make_pair(cur, prev);

      prev = cur;
      cur = next;
    }
  }
}

}  // namespace pplanner
