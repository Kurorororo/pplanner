#ifndef GRAPHPLAN_H_
#define GRAPHPLAN_H_

#include <array>
#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "domain/state.h"

namespace rwls {

struct GraphSchema {
  int goal_size;
  std::vector<int> is_goal;
  std::vector<int> goal_facts;
  std::vector<int> precondition_size;
  std::vector< std::vector<int> > precondition_map;
  std::vector< std::vector<int> > effect_map;
};

struct PlanningGraph {
  int n_layers;
  int goal_counter;
  std::vector<int> fact_layer_membership;
  std::vector<int> action_layer_membership;
  std::vector<int> precondition_counter;
  std::vector<bool> closed;
  std::vector<int> scheduled_facts;
  std::vector<int> scheduled_actions;
  std::vector< std::vector<int> > g_set;
  std::array<std::vector<bool>, 2> marked;

  PlanningGraph() {}

  ~PlanningGraph() {}
};

void ResetGraph(PlanningGraph *graph);

void InitializeSchema(const Domain &domain, GraphSchema *schema);

void InitializeGraph(const Domain &domain, const GraphSchema &schema,
                     PlanningGraph *graph);

std::vector<int> Graphplan(const State &initial, const Domain &domain,
                           const GraphSchema &schema, PlanningGraph *graph,
                           std::unordered_set<int> &preferred);

int GraphplanCost(const State &initial, const Domain &domain,
                  const GraphSchema &schema, PlanningGraph *graph);

int GraphplanCost(const State &initial, const Domain &domain,
                  const GraphSchema &schema, PlanningGraph *graph,
                  std::unordered_set<int> &preferred);

void ConstructRRPG(const State &initial, const Domain &domain,
                   const GraphSchema &schema,
                   const std::unordered_set<int> &black_list,
                   PlanningGraph *graph);

} // namespace rwls

#endif // GRAPHPLAN_H_
