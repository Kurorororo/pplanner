#ifndef LOCK_FREE_CLOSED_LIST_H_
#define LOCK_FREE_CLOSED_LIST_H_

#include <atomic>
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

class LockFreeClosedList {
 public:
  LockFreeClosedList(int exponent = 26)
      : mask_((1u << exponent) - 1), closed_(1 << exponent) {
    Init();
  }

  bool IsClosed(uint32_t hash, const std::vector<uint32_t>& packed_state) const;

  bool Close(std::shared_ptr<SearchNodeWithNext> node);

  std::pair<std::shared_ptr<SearchNodeWithNext>,
            std::shared_ptr<SearchNodeWithNext> >
  Find(std::size_t head_index, const std::vector<uint32_t>& packed_state) const;

  void Dump(std::shared_ptr<const SASPlus> problem,
            std::shared_ptr<const StatePacker> packer) const;

 private:
  void Init();

  uint32_t mask_;
  std::vector<std::shared_ptr<SearchNodeWithNext> > closed_;
};

}  // namespace pplanner

#endif  // LOCK_FREE_CLOSED_LIST_H_
