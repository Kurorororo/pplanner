#ifndef SIMPLIFIED_GRAPHPLAN_H_
#define SIMPLIFIED_GRAPHPLAN_H_

#include <array>
#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "domain/relaxed_domain.h"
#include "domain/state.h"
#include "heuristic/graphplan.h"

namespace rwls {

void InitializeGraph(const RelaxedDomain &domain, PlanningGraph *graph);

std::vector<int> Graphplan(const std::vector<int> &initial,
                           const RelaxedDomain &domain, PlanningGraph *graph,
                           std::unordered_set<int> &preferred);

int GraphplanCost(const std::vector<int> &initial, const RelaxedDomain &domain,
                  PlanningGraph *graph);

int GraphplanCost(const std::vector<int> &initial, const RelaxedDomain &domain,
                  PlanningGraph *graph, std::unordered_set<int> &preferred);

void ConstructRRPG(const std::vector<int> &initial, const RelaxedDomain &domain,
                   const std::unordered_set<int> &black_list,
                   PlanningGraph *graph);

} // namespace rwls

#endif // SIMPLIFIED_GRAPHPLAN_H_
