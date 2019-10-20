#ifndef LOCK_FREE_CLOSED_LIST_H_
#define LOCK_FREE_CLOSED_LIST_H_

#include <atomic>
#include <fstream>
#include <unordered_set>
#include <utility>
#include <vector>

#include "sas_plus.h"
#include "search_graph/state_packer.h"
#include "search_node.h"

namespace pplanner {

struct SearchNodeWithNext : public SearchNode {
  std::shared_ptr<SearchNodeWithNext> next;
};

template <typename T = SearchNodeWithNext>
class LockFreeClosedList {
 public:
  LockFreeClosedList(int exponent = 26)
      : mask_((1u << exponent) - 1), closed_(1 << exponent) {
    Init();
  }

  bool IsClosed(uint32_t hash,
                const std::vector<uint32_t>& packed_state) const {
    return Find(hash, packed_state) != nullptr;
  }

  bool Close(std::shared_ptr<T> node);

  std::shared_ptr<T> Find(uint32_t hash,
                          const std::vector<uint32_t>& packed_state) const;

  std::pair<std::shared_ptr<T>, std::shared_ptr<T> > Find(
      std::size_t head_index, const std::vector<uint32_t>& packed_state) const;

  void Dump(std::shared_ptr<const SASPlus> problem,
            std::shared_ptr<const StatePacker> packer) const;

 private:
  void Init();

  uint32_t mask_;
  std::vector<std::shared_ptr<T> > closed_;
};

/***
 * Michael, M., 2002.
 * High Performance Dynamic Lock-Free Hash Tables and List-Based Sets
 * In state space search, there is no need for deletion of nodes in closed
 * lists, so we did not implement marking and deletion.
 * In addition, it is dirty that each mark uses the lowest bit of the pointer.
 */

template <typename T>
void LockFreeClosedList<T>::Init() {
  for (int i = 0, n = closed_.size(); i < n; ++i) closed_[i] = nullptr;
}

template <typename T>
bool LockFreeClosedList<T>::Close(std::shared_ptr<T> node) {
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

template <typename T>
std::shared_ptr<T> LockFreeClosedList<T>::Find(
    uint32_t hash, const std::vector<uint32_t>& packed_state) const {
  std::size_t i = hash & mask_;
  auto p = Find(i, packed_state);

  if (p.first == nullptr) return nullptr;

  return packed_state == p.first->packed_state ? p.first : nullptr;
}

template <typename T>
std::pair<std::shared_ptr<T>, std::shared_ptr<T> > LockFreeClosedList<T>::Find(
    std::size_t head_index, const std::vector<uint32_t>& packed_state) const {
  while (true) {
    std::shared_ptr<T> prev = closed_[head_index];

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

template <typename T>
void LockFreeClosedList<T>::Dump(
    std::shared_ptr<const SASPlus> problem,
    std::shared_ptr<const StatePacker> packer) const {
  std::ofstream expanded_nodes;
  expanded_nodes.open("expanded_nodes.csv", std::ios::out);
  expanded_nodes << "node_id,parent_node_id,h,action,timestamp";

  for (int i = 0; i < problem->n_variables(); ++i) expanded_nodes << ",v" << i;

  expanded_nodes << std::endl;

  std::vector<int> state(problem->n_variables());

  for (auto list : closed_) {
    std::shared_ptr<T> node = list;

    while (node != nullptr) {
      int node_id = node->id;
      int parent_id = node->parent == nullptr ? -1 : node->parent->id;
      int h = node->h;
      expanded_nodes << node_id << "," << parent_id << "," << h << ","
                     << node->action << "," << node_id;

      packer->Unpack(node->packed_state.data(), state);

      for (int j = 0; j < problem->n_variables(); ++j)
        expanded_nodes << "," << state[j];

      expanded_nodes << std::endl;

      node = node->next;
    }
  }
}

}  // namespace pplanner

#endif  // LOCK_FREE_CLOSED_LIST_H_
