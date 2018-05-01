#include "open_lists/single_open_list.h"

#include <vector>

#include "evaluator_factory.h"

namespace pplanner {

int SingleOpenList::Push(const vector<int> &state, int node, bool preferred)
  override {
  assert(evaluators_ != nullptr);

  values_.clear();

  for (auto evaluator : evaluators_)
    values_.push_back(evaluator->Evaluate(state, node));

  Push(values_, node, preferred);

  return values_[0];
}


} // namespace pplanner
