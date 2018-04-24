#ifndef RELAXED_DOMAIN_H_
#define RELAXED_DOMAIN_H_

#include <vector>

#include "domain/domain.h"

namespace rwls {

struct RelaxedDomain {
  size_t fact_size;
  size_t action_size;
  int goal_size;
  std::vector<int> ids;
  std::vector<int> costs;
  std::vector<int> precondition_size;
  std::vector<std::vector<int> > preconditions;
  std::vector<int> effects;
  std::vector<bool> is_goal;
  std::vector<int> goal;
  std::vector<std::vector<int> > precondition_map;
  std::vector<std::vector<int> > effect_map;
};

void InitializeRelaxedDomain(const Domain &domain, RelaxedDomain *r_domain);

} // namespace rwls

#endif // RELAXED_DOMAIN_H_
