#include "search/kgbfs.h"

namespace pplanner {

using std::unordered_set;
using std::vector;

int KGBFS::Search() {
  auto state = InitialExpand();
  vector<int> child(state);
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<int> nodes(k_);

  while (!NoNode()) {
    nodes.clear();

    for (int i=0; i<k_; ++i) {
      int node = NodeToExpand();

      if (node == -1)
        break;

      nodes.push_back(node);
    }

    for (auto node : nodes) {
      int goal = Expand(node, state, child, applicable, preferred);

      if (goal != -1) return goal;

      if (IsLimit()) return -1;
    }
  }

  return -1;
}

} // namespace pplanner
