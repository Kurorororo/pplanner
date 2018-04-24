#ifndef NEW_FACT_H_
#define NEW_FACT_H_

#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "domain/state.h"
#include "heuristic/preferring.h"

namespace rwls {

class NewFactPreferring : public Preferring {
 public:
  NewFactPreferring() {}

  NewFactPreferring(const Domain &domain) {
    variables_size_ = static_cast<int>(domain.variables_size);
    is_new_.resize(domain.fact_size, true);
    tmp_state_.resize(domain.variables_size);
  }

  void extract(const State &state, const Domain &domain,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred);

 private:
  int variables_size_;
  std::vector<bool> is_new_;
  State tmp_state_;
};

} // namespace rwls

#endif // NEW_FACT_H_
