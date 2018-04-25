#include "problem.h"

using std::vector;
using std::pair;

namespace pplanner {

void CopyPrecondition(int i, vector<pair<int, int> > &precondition) const {
  size_t size = preconditions_->SizeOfPartialState(i);
  partial_state.resize(size);

  auto var_iter = preconditions_->VarsBegin(i);
  auto value_iter = preconditions_->ValuesBegin(i);

  for (auto &p : partial_state) {
    p.first = *var_iter;
    p.second *value_iter;
    ++var_iter;
    ++value_iter;
  }
} // namespace pplanner
