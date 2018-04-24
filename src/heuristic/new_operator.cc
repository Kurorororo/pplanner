#include "heuristic/new_operator.h"

namespace rwls {

void NewOperatorPreferring::extract(const State &state, const Domain &domain,
                                    const std::vector<int> &applicable,
                                    std::unordered_set<int> &preferred) {
  preferred.clear();

  for (auto a : applicable) {
    if (is_new_[a]) {
      preferred.insert(a);
      is_new_[a] = false;
    }
  }
}

} // namespace rwls
