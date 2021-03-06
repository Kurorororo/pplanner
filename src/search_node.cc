#include "search_node.h"

namespace pplanner {

std::vector<int> ExtractPath(std::shared_ptr<const SearchNode> node) {
  if (node == nullptr) return std::vector<int>{-1};

  std::vector<int> result;

  while (node->parent != nullptr) {
    result.insert(result.begin(), node->action);
    node = node->parent;
  }

  return result;
}

}  // namespace pplanner
