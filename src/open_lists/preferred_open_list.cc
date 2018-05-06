#include "preferred_open_list.h"

namespace pplanner {

int PreferredOpenList::EvaluateAndPush(const std::vector<int> &state, int node,
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

int PreferredOpenList::Pop() {
  if (lists_[0]->IsEmpty() && lists_[1]->IsEmpty()) return -1;

  if (lists_[0]->IsEmpty()) {
    --priorities_[1];

    return lists_[1]->Pop();
  }

  if (lists_[1]->IsEmpty()) {
    --priorities_[0];

    return lists_[0]->Pop();
  }

  int arg_max = priorities_[0] > priorities_[1] ? 0 : 1;
  --priorities_[arg_max];

  return lists_[arg_max]->Pop();
}


} // namespace pplanner
