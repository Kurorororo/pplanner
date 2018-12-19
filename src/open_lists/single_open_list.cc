#include "open_lists/single_open_list.h"

#include <vector>

namespace pplanner {

using std::vector;

int SingleOpenList::EvaluateAndPush(const vector<int> &state, int node,
                                    bool preferred) {
  values_.clear();

  for (auto evaluator : evaluators_) {
    int value = evaluator->Evaluate(state, node);
    if (value == -1) return value;
    values_.push_back(value);
  }

  Push(values_, node, preferred);

  return values_[0];
}


} // namespace pplanner
