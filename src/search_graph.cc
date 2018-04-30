#include "search_graph.h"

#include <algorithm>

namespace pplanner {

std::vector<int> ExtractPath(const SearchGraph &graph, int node) {
  if (node == -1) return std::vector<int>{-1};
  std::vector<int> result;

  while (graph.Parent(node) != -1) {
    result.push_back(graph.Action(node));
    node = graph.Parent(node);
  }

  std::reverse(result.begin(), result.end());

  return result;
}

} // namespace pplanner
