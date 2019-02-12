#include "multithread_search/closed_list.h"

#include <iostream>

namespace pplanner {

bool ClosedList::IsClosed(uint32_t hash,
                          const std::vector<uint32_t> &packed_state) const {
  std::size_t i = hash & closed_mask_;

  while (closed_[i] != nullptr) {
    if (packed_state == closed_[i]->packed_state) return true;

    i = (i == closed_.size() - 1) ? 0 : i + 1;
  }

  return false;
}

bool ClosedList::Close(SearchNode* node) {
  std::size_t i = node->hash & closed_mask_;

  while (closed_[i] != nullptr) {
    if (node->packed_state == closed_[i]->packed_state) return false;

    i = (i == closed_.size() - 1) ? 0 : i + 1;
  }

  closed_[i] = node;
  ++n_closed_;

  if (2 * n_closed_ > closed_.size()) Resize();

  return true;
}

void ClosedList::Clear() {
  n_closed_ = 0;
  closed_.clear();
}

void ClosedList::Resize() {
  int closed_exponent = 1;

  while ((1u << closed_exponent) < 3 * n_closed_)
    ++closed_exponent;

  uint32_t closed_mask = (1u << closed_exponent) - 1;
  std::vector<SearchNode*> new_closed(1 << closed_exponent, nullptr);

  for (int k = 0, m = closed_.size(); k < m; ++k) {
    auto node = closed_[k];

    if (node != nullptr) {
      std::size_t i = node->hash & closed_mask;

      while (new_closed[i] != nullptr)
        i = i == (new_closed.size() - 1) ? 0 : i + 1;

      new_closed[i] = node;
    }
  }

  closed_mask_ = closed_mask;
  closed_.swap(new_closed);
}

} // namespace pplanner
