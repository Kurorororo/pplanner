#include "open_list.h"

#include <vector>

#include "evaluator_factory.h"

namespace pplanner {

int OpenList::Push(const vector<int> &state, int node) {
  assert(evaluators_ != nullptr);

  values_.clear();

  for (auto evaluator : evaluators_)
    values_.push_back(evaluator->Evaluate(state, node));

  Push(values_, node);

  return values_[0];
}


} // namespace pplanner
