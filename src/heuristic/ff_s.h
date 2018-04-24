#ifndef FF_S_H_
#define FF_S_H_

#include <limits>
#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "heuristic/simplified_graphplan.h"
#include "heuristic/heuristic.h"

namespace rwls {

class FFS : public HeuristicInterface<FFS> {
 public:
  void Initialize(const Domain &domain) {
    InitializeRelaxedDomain(domain, &r_domain_);
    InitializeGraph(r_domain_, &graph_);
  }

  inline int operator()(const State &state, const Domain &domain,
                        const std::vector<int> &applicable,
                        std::unordered_set<int> &preferred) {
    if (applicable.empty()) return std::numeric_limits<int>::max();
    StateToFactSet(state, domain, facts_);

    return GraphplanCost(facts_, r_domain_, &graph_, preferred);
  }

  inline int operator()(int parent, int node, const State &state,
                        const Domain &domain) {
    StateToFactSet(state, domain, facts_);

    return GraphplanCost(facts_, r_domain_, &graph_);
  }

  inline int operator()(const State &state, const Domain &domain) {
    StateToFactSet(state, domain, facts_);

    return GraphplanCost(facts_, r_domain_, &graph_);
  }

 private:
  State state_;
  std::vector<int> facts_;
  RelaxedDomain r_domain_;
  PlanningGraph graph_;
};

} // namespace rwls

#endif // FF_S_H_
