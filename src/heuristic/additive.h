#ifndef ADDITIVE_H_
#define ADDITIVE_H_

#include <queue>
#include <limits>
#include <utility>
#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "domain/relaxed_domain.h"
#include "heuristic/heuristic.h"

namespace rwls {

struct AdditiveTable {
  int goal_counter;
  std::vector<int> op_cost;
  std::vector<int> precondition_counter;
  std::vector<int> prop_cost;
  std::vector<int> best_support;
};

struct FirstGreater {
  bool operator()(const std::pair<int, int> &a, const std::pair<int, int> &b) {
    return a.first > b.first;
  }
};

using PQueue = std::priority_queue<std::pair<int, int>,
                                   std::vector<std::pair<int, int> >,
                                   FirstGreater>;

void InitializeAdditiveTable(const RelaxedDomain &domain,
                             AdditiveTable *table);

void GeneralizedDijkstra(const std::vector<int> &state,
                         const RelaxedDomain &domain, PQueue &q,
                         AdditiveTable *table);

int AdditiveCost(const std::vector<int> &goal, const AdditiveTable &table);

class Additive : public HeuristicInterface<Additive> {
 public:
  void Initialize(const Domain &domain) {
    state_.resize(domain.variables_size);
    facts_.resize(domain.fact_size);
    InitializeRelaxedDomain(domain, &r_domain_);
    InitializeAdditiveTable(r_domain_, &table_);
  }

  inline int operator()(const State &state, const Domain &domain,
                        const std::vector<int> &applicable,
                        std::unordered_set<int> &preferred) {
    if (applicable.empty()) return std::numeric_limits<int>::max();

    StateToFactSet(state, domain, facts_);
    GeneralizedDijkstra(facts_, r_domain_, q_, &table_);

    return AdditiveCost(r_domain_.goal, table_);
  }

  inline int operator()(const State &state, const Domain &domain) {
    StateToFactSet(state, domain, facts_);
    GeneralizedDijkstra(facts_, r_domain_, q_, &table_);

    return AdditiveCost(r_domain_.goal, table_);
  }

 private:
  State state_;
  std::vector<int> facts_;
  PQueue q_;
  AdditiveTable table_;
  RelaxedDomain r_domain_;
};


} // namespace rwls

#endif // ADDITIVE_H_
