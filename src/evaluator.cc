#include "evaluator.h"

namespace pplanner {

Evaluator::~Evaluator() {}

int Evaluate(const std::vector<std::shared_ptr<Evaluator> > evaluators,
             const std::vector<int> &state, int node,
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
