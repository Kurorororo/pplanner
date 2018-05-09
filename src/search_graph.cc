#include "search_graph.h"

#include <algorithm>

namespace pplanner {

int SearchGraph::GenerateNodeIfNotClosed(const std::vector<int> &state,
                                         int parent, int action,
                                         bool is_preferred) {
  int node = states_->AddIfNotClosed(state);

  if (node != -1) {
    parents_.push_back(parent);
    actions_.push_back(action);
  }

  return node;
}

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
