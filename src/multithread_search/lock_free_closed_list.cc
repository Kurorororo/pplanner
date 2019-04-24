#include "multithread_search/lock_free_closed_list.h"

#include <fstream>

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

void LockFreeClosedList::Dump(std::shared_ptr<const SASPlus> problem,
                              std::shared_ptr<const StatePacker> packer) const {
  std::ofstream expanded_nodes;
  expanded_nodes.open("expanded_nodes.csv", std::ios::out);
  expanded_nodes << "node_id,parent_node_id,h";

  for (int i = 0; i < problem->n_variables(); ++i) expanded_nodes << ",v" << i;

  std::vector<int> state(problem->n_variables());

  for (auto list : closed_) {
    std::shared_ptr<SearchNodeWithNext> node = list;

    while (node != nullptr) {
      int node_id = node->id;
      int parent_id = node->parent == nullptr ? -1 : node->parent->id;
      int h = node->h;
      expanded_nodes << node_id << "," << parent_id << "," << h;

      packer->Unpack(node->packed_state.data(), state);

      for (int j = 0; j < problem->n_variables(); ++j)
        expanded_nodes << "," << state[j];

      expanded_nodes << std::endl;

      node = node->next;
    }
  }
}

}  // namespace pplanner
