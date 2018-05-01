#include "heuristics/blind.h"

namespace pplanner {

void Blind::Init() {
  assert(problem_ != nullptr);

  int cheapest = -1;

  for (int i=0, n=problem_->n_actions(); i<n; ++i) {
    int cost = problem_->ActionCost(i);
    if (cost < cheapest || cheapest == -1) cheapest = cost;
  }

  cheapest_ = cheapest;
}


} // namespace pplanner
