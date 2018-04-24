#ifndef NEW_OPERATOR_H_
#define NEW_OPERATOR_H_

#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "domain/state.h"
#include "heuristic/preferring.h"

namespace rwls {

class NewOperatorPreferring : public Preferring {
 public:
  NewOperatorPreferring() {}

  NewOperatorPreferring(const Domain &domain) {
    is_new_.resize(domain.action_size, true);
  }

  void extract(const State &state, const Domain &domain,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred);

 private:
  std::vector<bool> is_new_;
};

} // namespace rwls

#endif // NEW_OPERATOR_H_
