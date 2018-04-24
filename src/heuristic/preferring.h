#ifndef PREFERRING_H_
#define PREFERRING_H_

#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "domain/state.h"

namespace rwls {

class Preferring {
 public:
  virtual void extract(const State &state, const Domain &domain,
                       const std::vector<int> &applicable,
                       std::unordered_set<int> &preferred) = 0;
  virtual ~Preferring() {}
};

} // namespace rwls

#endif // PREFERRING_H_
