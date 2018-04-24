#include "heuristic/new_fact.h"

namespace rwls {

void NewFactPreferring::extract(const State &state, const Domain &domain,
                                const std::vector<int> &applicable,
                                std::unordered_set<int> &preferred) {
  preferred.clear();

  for (auto a : applicable) {
    tmp_state_ = state;

    ApplyEffect(domain.effects[a], tmp_state_);

    for (int i=0; i<variables_size_; ++i) {
      int f = ToFact(domain.fact_offset, i, tmp_state_[i]);

      if (is_new_[f]) {
        preferred.insert(a);
        is_new_[f] = false;
        break;
      }
    }
  }
}

} // namespace rwls
