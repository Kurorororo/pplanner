#include "multithread_search/independent_context.h"

namespace pplanner {

void IndependentContext::UpdatePriority(int plateau_threshold) {
  priority_[0] = n_plateau_ / plateau_threshold;

  for (int i = 0, n = open_list_->MinimumValue().size(); i < n; ++i)
    priority_[i + 1] = open_list_->MinimumValue()[i];
}

}  // namespace pplanner