#ifndef CLOSED_LIST_H_
#define CLOSED_LIST_H_

#include <cstdint>

#include <memory>
#include <vector>

#include "multithread_search/search_node.h"

namespace pplanner {

class ClosedList {
 public:
  ClosedList(int closed_exponent=22)
    : closed_exponent_(closed_exponent),
      closed_mask_((1u << closed_exponent) - 1),
      n_closed_(0),
      closed_(1 << closed_exponent, nullptr) {}

  bool IsClosed(uint32_t hash, const std::vector<uint32_t> &packed_state) const;

  void Close(std::shared_ptr<SearchNode> node);

  void Clear();

 private:
  std::size_t Find(uint32_t hash,
                   const std::vector<uint32_t> &packed_state) const;

  void Resize();

  int closed_exponent_;
  uint32_t closed_mask_;
  std::size_t n_closed_;
  std::vector<std::shared_ptr<SearchNode> > closed_;
};

} // namespace pplanner

#endif // CLOSED_LIST_H_
