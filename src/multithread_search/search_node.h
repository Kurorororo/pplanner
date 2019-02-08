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
  std::shared_ptr<SearchNode> parent;
  std::vector<uint32_t> packed_state;
  std::vector<uint8_t> landmark;
};

struct SearchNodeWithHash : public SearchNode {
  uint32_t hash2;
};

std::vector<int> ExtractPath(std::shared_ptr<const SearchNode> node);

}


#endif // SEARCH_NODE_H_
