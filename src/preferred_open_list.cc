#include "preferred_open_list.h"

namespace pplanner {

int PreferredOpenList::Push(const std::vector<int> &state, int node,
                            bool preferred=false) {
  values_.clear();

  for (auto evaluator : evaluators_)
    values_.push_back(evaluator->Evaluate(state, node));

  Push(values_, node, preferred);

  return values_[0];
}

int PreferredOpenList::Pop() {
  if (lists_[0]->IsEmpty && lists_[1]->IsEmpty()) return -1;

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
