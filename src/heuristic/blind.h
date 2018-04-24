#ifndef BLIND_H_
#define BLIND_H_

#include <limits>
#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "heuristic/heuristic.h"

namespace rwls {

class Blind : public HeuristicInterface<Blind> {
 public:
  void Initialize(const Domain &domain) {
    cheapest_ = *min_element(domain.costs.begin(), domain.costs.end());
  }

  inline int operator()(const State &state, const Domain &domain,
                        const std::vector<int> &applicable,
                        std::unordered_set<int> &preferred) {
    preferred.clear();

    if (applicable.empty()) return std::numeric_limits<int>::max();

    if (GoalCheck(domain.goal, state)) return 0;

    return cheapest_;
  }

  int operator()(const State &state, const Domain &domain) {
    if (GoalCheck(domain.goal, state)) return 0;

    return cheapest_;
  }

 private:
  int cheapest_;
};

} // namespace rwls

#endif // BLIND_H_
