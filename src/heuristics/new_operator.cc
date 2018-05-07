#include "heuristics/new_operator.h"

namespace pplanner {

using std::unordered_set;
using std::vector;

int NewOperator::Evaluate(const vector<int> &state, int node,
                          const vector<int> &applicable,
                          unordered_set<int> &preferred) {
  preferred.clear();

  for (auto o : applicable) {
    if (is_new_[o]) {
      is_new_[o] = false;
      preferred.insert(o);
    }
  }

  return 1;
}

} // namespace pplanner
