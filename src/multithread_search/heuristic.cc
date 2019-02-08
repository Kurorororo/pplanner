#include "multithread_search/heuristic.h"

namespace pplanner {

Heuristic::~Heuristic() {}

int Evaluate(const std::vector<std::shared_ptr<Heuristic> > evaluators,
             const std::vector<int> &state, std::shared_ptr<SearchNode> node,
             std::vector<int> &values) {
  values.clear();

  for (auto e : evaluators) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}
} // namespace pplanner
