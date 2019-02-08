#include "multithread_search/closed_list.h"

namespace pplanner {

bool ClosedList::IsClosed(uint32_t hash,
                          const std::vector<uint32_t> &packed_state) const {
  return closed_[Find(hash, packed_state)] != nullptr;
}

void ClosedList::Close(std::shared_ptr<SearchNode> node) {
  std::size_t i = Find(node->hash, node->packed_state);

  if (closed_[i] != nullptr) return;

  ++n_closed_;
  closed_[i] = node;

  if (2 * (n_closed_ + 1) > closed_.size())
    Resize();
}

std::size_t ClosedList::Find(uint32_t hash,
                             const std::vector<uint32_t> &packed_state) const {
  std::size_t i = hash & closed_mask_;

  while (closed_[i] != nullptr) {
    if (packed_state == closed_[i]->packed_state) break;

    i = (i == closed_.size() - 1) ? 0 : i + 1;
  }

  return i;
}

void ClosedList::Clear() {
  n_closed_ = 0;
  closed_.clear();
}

void ClosedList::Resize() {
  closed_exponent_ = 1;

  while ((1u << closed_exponent_) < 3 * n_closed_)
    ++closed_exponent_;

  closed_mask_ = (1u << closed_exponent_) - 1;
  std::vector<std::shared_ptr<SearchNode> > new_closed(
      1 << closed_exponent_, nullptr);

  for (int k = 0, m = closed_.size(); k < m; ++k) {
    auto node = closed_[k];

    if (node != nullptr) {
      std::size_t i = node->hash & closed_mask_;

      while (new_closed[i] != nullptr)
        i = i == (new_closed.size() - 1) ? 0 : i + 1;

      new_closed[i] = node;
    }
  }

  closed_.swap(new_closed);
}

} // namespace pplanner
