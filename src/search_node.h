#ifndef SEARCH_NODE_H_
#define SEARCH_NODE_H_

#include <cstdint>

#include <memory>
#include <vector>

namespace pplanner {

struct SearchNode {
  int cost;
  int h;
  int action;
  int id;
  uint32_t hash;
  std::shared_ptr<SearchNode> parent;
  std::vector<uint32_t> packed_state;
  std::vector<uint8_t> landmark;
};

std::vector<int> ExtractPath(const std::shared_ptr<const SearchNode> node);

}  // namespace pplanner

#endif  // SEARCH_NODE_H_
