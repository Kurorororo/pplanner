#include "search/pigbfs.h"

namespace pplanner {

int PIGBFS::Search() {
  auto state = InitialEvaluate();

  if (!NoNode()) {
    while (n_open_nodes() < world_size() && !NoNode()) {
      int node = Pop();
      int goal = IndependentExpand(node, state);

      if (goal != -1) {
        SendTermination();

        return goal;
      }
    }

    Distribute();
  }

  while (!ReceiveTermination()) {
    ReceiveNodes();
    if (NoNode()) continue;

    int node = Pop();
    int goal = IndependentExpand(node, state);

    if (goal != -1) {
      SendTermination();

      return goal;
    }
  }

  return -1;
}

} // namespace pplanner
