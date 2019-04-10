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
  uint32_t hash;
  SearchNode *parent;
  std::vector<uint32_t> packed_state;
  std::vector<uint8_t> landmark;
};

std::vector<int> ExtractPath(const SearchNode *node);

} // namespace pplanner


#endif // SEARCH_NODE_H_
