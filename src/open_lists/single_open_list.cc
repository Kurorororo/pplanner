#include "open_lists/single_open_list.h"

#include <vector>

namespace pplanner {

using std::vector;

int SingleOpenList::EvaluateAndPush(const vector<int> &state, int node,
                                    bool preferred) {
  assert(evaluators_ != nullptr);

  values_.clear();

  for (auto evaluator : evaluators_)
    values_.push_back(evaluator->Evaluate(state, node));

  Push(values_, node, preferred);

  return values_[0];
}


} // namespace pplanner
