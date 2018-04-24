#ifndef FF_ADD_H_
#define FF_ADD_H_

#include <limits>
#include <queue>
#include <utility>
#include <unordered_set>
#include <vector>
#include <iostream>
#include "domain/domain.h"
#include "domain/relaxed_domain.h"
#include "heuristic/additive.h"
#include "heuristic/heuristic.h"
#include "heuristic/preferring.h"

namespace rwls {

struct RpgSet {
  std::vector<bool> plan;
  std::vector<bool> marked;
};

void InitializeRpgSet(const Domain &domain, RpgSet *rpg);

int PlanCost(const Domain &domain, const RelaxedDomain &r_domain,
             const AdditiveTable &table, RpgSet *rpg);

int PlanCost(const Domain &domain, const RelaxedDomain &r_domain,
             const AdditiveTable &table, const std::vector<int> &applicable,
             std::unordered_set<int> &preferred, RpgSet *rpg);

class FFAdd : public HeuristicInterface<Additive> {
 public:
  void Initialize(const Domain &domain) {
    state_.resize(domain.variables_size);
    facts_.resize(domain.fact_size);
    InitializeRelaxedDomain(domain, &r_domain_);
    InitializeAdditiveTable(r_domain_, &table_);
    InitializeRpgSet(domain, &rpg_);
  }

  inline int operator()(const State &state, const Domain &domain) {
    StateToFactSet(state, domain, facts_);
    GeneralizedDijkstra(facts_, r_domain_, q_, &table_);

    int h_add = AdditiveCost(r_domain_.goal, table_);

    if (h_add == std::numeric_limits<int>::max()) return h_add;

    return PlanCost(domain, r_domain_, table_, &rpg_);
  }

  inline int operator()(const State &state, const Domain &domain,
                        const std::vector<int> &applicable,
                        std::unordered_set<int> &preferred) {
    if (applicable.empty()) return std::numeric_limits<int>::max();

    StateToFactSet(state, domain, facts_);
    GeneralizedDijkstra(facts_, r_domain_, q_, &table_);

    int h_add = AdditiveCost(r_domain_.goal, table_);

    if (h_add == std::numeric_limits<int>::max()) return h_add;

    return PlanCost(domain, r_domain_, table_, applicable, preferred, &rpg_);
  }

 private:
  State state_;
  std::vector<int> facts_;
  PQueue q_;
  AdditiveTable table_;
  RelaxedDomain r_domain_;
  RpgSet rpg_;
};

class FFAddPreferring : public Preferring {
 public:
  FFAddPreferring() {}

  FFAddPreferring(const Domain &domain) {
    facts_.resize(domain.fact_size);
    InitializeRelaxedDomain(domain, &r_domain_);
    InitializeAdditiveTable(r_domain_, &table_);
    InitializeRpgSet(domain, &rpg_);
  }

  void extract(const State &state, const Domain &domain,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) {
    StateToFactSet(state, domain, facts_);
    GeneralizedDijkstra(facts_, r_domain_, q_, &table_);

    int h_add = AdditiveCost(r_domain_.goal, table_);

    if (h_add == std::numeric_limits<int>::max()) return;

    PlanCost(domain, r_domain_, table_, applicable, preferred, &rpg_);
  }

 private:
  std::vector<int> facts_;
  PQueue q_;
  AdditiveTable table_;
  RelaxedDomain r_domain_;
  RpgSet rpg_;
};

} // namespace rwls

#endif // FF_ADD_H_
