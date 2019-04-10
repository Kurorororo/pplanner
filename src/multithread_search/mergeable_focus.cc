#include "mergeable_focus.h"

namespace pplanner {

SearchNode* MergeableFocus::Pop() {
  if (arg_min_ == -1) return nullptr;

  auto node = open_lists_[arg_min_]->Pop();

  if (open_lists_.size() == 1) return node;

  if (arg_min > 0 && open_lists_[arg_min_]->IsEmpty())
    open_lists_.erase(open_lists_.begin() + arg_min);

  minimum_values_.clear();
  arg_min_ = -1;

  for (int i = 0, n = open_lists_.size(); i < n; ++i) {
    if (open_lists_[i]->IsEmpty()) continue;

    if (arg_min_ == -1 || open_lists_[i]->MinimumValue() < minimum_values_) {
      minimum_values_ = ptr->MinimumValue();
      arg_min_ = i;
    }
  }

  return node;
}

void MergeableFocus::Merge(std::shared_ptr<MergeableFocus> focus) {
  best_h_ = best_h_ < focus->best_h_ : best_h_ : focus->best_h_;

  if (focus->IsEmpty()) return;

  if (IsEmpty() || focus->MinimumValue() < MinimumValue()) {
    arg_min_ = open_lists_.size() + focus->arg_min_;
    minimum_values_ = focus->minimum_values_;
  }

  open_lists_.insert(open_lists_.end(), focus->open_lists_.begin(),
                     focus->open_lists_.end());
}

}  // namespace pplanner
