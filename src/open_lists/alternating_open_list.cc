#include "open_lists/alternating_open_list.h"

#include <vector>

namespace pplanner {

using std::vector;

std::size_t AlternatingOpenList::size() const {
  std::size_t size = 0;

  for (int i = 0, n = evaluators_.size(); i < n; ++i)
    size += lists_[i]->size();

  return size;
}

void AlternatingOpenList::Push(vector<int> &values, int node,
                               bool preferred) {
  for (int i = 0, n = values.size(); i < n; ++i)
    lists_[i]->Push(vector<int>{values[i]}, node);
}

int AlternatingOpenList::EvaluateAndPush(const vector<int> &state, int node,
                                         bool preferred) {
  thread_local vector<int> values;

  values.clear();

  for (auto evaluator : evaluators_) {
    int value = evaluator->Evaluate(state, node);
    if (value == -1) return value;
    values.push_back(value);
  }

  Push(values, node, preferred);

  return values[0];
}

int AlternatingOpenList::Pop() {
  for (int i = 0, n = evaluators_.size(); i < n; ++i) {
    if (!lists_[idx_]->IsEmpty())
      break;

    idx_ = (idx_ + 1) % evaluators_.size();
  }

  int node = lists_[idx_]->Pop();
  idx_ = (idx_ + 1) % evaluators_.size();

  return node;
}

bool AlternatingOpenList::IsEmpty() const {
  for (int i = 0, n = evaluators_.size(); i < n; ++i)
    if (!lists_[i]->IsEmpty())
      return false;

  return true;
}

const vector<int>& AlternatingOpenList::MinimumValues() const {
  thread_local vector<int> values;

  values.clear();

  for (int i = 0, n = values.size(); i < n; ++i)
    values.push_back(lists_[i]->MinimumValue(0));

  return values;
}

void AlternatingOpenList::Clear() {
  for (int i = 0, n = evaluators_.size(); i < n; ++i)
    lists_[idx_]->Clear();
}

} // namespace pplanner
