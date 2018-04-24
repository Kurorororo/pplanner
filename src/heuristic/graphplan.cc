#include "heuristic/graphplan.h"

#include <cassert>

#include <limits>

using std::unordered_set;
using std::vector;

namespace rwls {

void InitializeSchema(const Domain &domain, GraphSchema *schema) {
  schema->goal_size = domain.goal.size();
  size_t n_facts = domain.fact_size;
  schema->is_goal.resize(n_facts);
  std::fill(schema->is_goal.begin(), schema->is_goal.end(), 0);
  for (auto v : domain.goal) {
    int f = ToFact(domain.fact_offset, v);
    schema->is_goal[f] = 1;
    schema->goal_facts.push_back(f);
  }

  schema->precondition_map.resize(n_facts);
  schema->effect_map.resize(n_facts);
  size_t n_actions = domain.preconditions.size();
  schema->precondition_size.resize(n_actions);
  for (size_t i=0; i<n_actions; ++i) {
    schema->precondition_size[i] = domain.preconditions[i].size();
    for (auto v : domain.preconditions[i])
      schema->precondition_map[ToFact(domain.fact_offset, v)].push_back(i);
  }
  for (size_t i=0; i<n_actions; ++i)
    for (auto v : domain.effects[i])
      schema->effect_map[ToFact(domain.fact_offset, v)].push_back(i);
}

void InitializeGraph(const Domain &domain, const GraphSchema &schema,
                     PlanningGraph *graph) {
  size_t n_facts = domain.fact_size;
  graph->fact_layer_membership.resize(n_facts);
  graph->closed.resize(n_facts);
  graph->marked[0].resize(n_facts);
  graph->marked[1].resize(n_facts);
  int n_actions = schema.precondition_size.size();
  graph->precondition_counter.resize(n_actions);
  graph->action_layer_membership.resize(n_actions);
}

void ResetGraph(PlanningGraph *graph) {
  graph->n_layers = 0;
  graph->goal_counter = 0;
  std::fill(graph->fact_layer_membership.begin(),
            graph->fact_layer_membership.end(), -1);
  std::fill(graph->action_layer_membership.begin(),
            graph->action_layer_membership.end(), -1);
  std::fill(graph->closed.begin(), graph->closed.end(), 0);
  std::fill(graph->precondition_counter.begin(),
            graph->precondition_counter.end(), 0);
  graph->scheduled_facts.clear();
  graph->scheduled_actions.clear();
  for (auto &g : graph->g_set)
    g.clear();
}

int FactLayer(const GraphSchema &schema, PlanningGraph *graph) {
  while (!graph->scheduled_facts.empty()) {
    int f = graph->scheduled_facts.back();
    graph->scheduled_facts.pop_back();
    graph->fact_layer_membership[f] = graph->n_layers;
    if (schema.is_goal[f] == 1 && ++graph->goal_counter == schema.goal_size)
      return 1;
    for (auto o : schema.precondition_map[f]) {
      if (++graph->precondition_counter[o] == schema.precondition_size[o])
        graph->scheduled_actions.push_back(o);
    }
  }
  return 0;
}

void ActionLayer(const Domain &domain, const GraphSchema &schema,
                 PlanningGraph *graph)  {
  while (!graph->scheduled_actions.empty()) {
    int o = graph->scheduled_actions.back();
    graph->scheduled_actions.pop_back();
    graph->action_layer_membership[o] = graph->n_layers;
    for (auto v : domain.effects[o]) {
      int f = ToFact(domain.fact_offset, v);
      if (graph->closed[f] == 0) {
        graph->closed[f] = 1;
        graph->scheduled_facts.push_back(f);
      }
    }
  }
}

void ConstructGraph(const State &initial, const Domain &domain,
                    const GraphSchema &schema, PlanningGraph *graph) {
  ResetGraph(graph);
  for (int i=0, n=static_cast<int>(domain.variables_size); i<n; ++i) {
    int f = ToFact(domain.fact_offset, i, initial[i]);
    graph->closed[f] = 1;
    graph->scheduled_facts.push_back(f);
  }
  while (!graph->scheduled_facts.empty()) {
    int is_end = FactLayer(schema, graph);
    if (is_end == 1) {
      ++graph->n_layers;
      return;
    }
    ActionLayer(domain, schema, graph);
    ++graph->n_layers;
  }
  graph->n_layers = -1;
}

void RistrictedFactLayer(const GraphSchema &schema,
                         const unordered_set<int> &black_list,
                         PlanningGraph *graph) {
  while (!graph->scheduled_facts.empty()) {
    int f = graph->scheduled_facts.back();
    graph->scheduled_facts.pop_back();
    graph->fact_layer_membership[f] = graph->n_layers;
    for (auto o : schema.precondition_map[f]) {
      if (++graph->precondition_counter[o] == schema.precondition_size[o]
          && black_list.find(o) == black_list.end())
        graph->scheduled_actions.push_back(o);
    }
  }
}

void ConstructRRPG(const State &initial, const Domain &domain,
                   const GraphSchema &schema,
                   const unordered_set<int> &black_list,
                   PlanningGraph *graph) {
  ResetGraph(graph);
  for (int i=0, n=static_cast<int>(domain.variables_size); i<n; ++i) {
    int f = ToFact(domain.fact_offset, i, initial[i]);
    graph->closed[f] = 1;
    graph->scheduled_facts.push_back(f);
  }
  while (!graph->scheduled_facts.empty()) {
    RistrictedFactLayer(schema, black_list, graph);
    ActionLayer(domain, schema, graph);
    ++graph->n_layers;
  }
}

int ChooseAction(int index, int i, const Domain &domain,
                 const GraphSchema &schema, const PlanningGraph &graph) {
  int min = -1;
  int argmin = 0;
  for (auto o : schema.effect_map[index]) {
    if (graph.action_layer_membership[o] != i-1) continue;
    int difficulty = 0;
    for (auto p : domain.preconditions[o])
      difficulty += graph.fact_layer_membership[ToFact(domain.fact_offset, p)];
    if (difficulty < min || min == -1) {
      min = difficulty;
      argmin = o;
    }
  }
  assert(-1 != min);
  return argmin;
}

void InitializeGSet(const GraphSchema &schema, PlanningGraph *graph) {
  graph->g_set.resize(graph->n_layers);
  for (auto g : schema.goal_facts)
    graph->g_set[graph->fact_layer_membership[g]].push_back(g);
}


int ExtractAction(int i, int g, const Domain &domain,
                  const GraphSchema &schema, PlanningGraph *graph) {
  int o = ChooseAction(g, i, domain, schema, *graph);
  for (auto v : domain.preconditions[o]) {
    int f = ToFact(domain.fact_offset, v);
    int j = graph->fact_layer_membership[f];
    if (j != 0 && graph->marked[0][f] == 0)
      graph->g_set[j].push_back(f);
  }
  for (auto v : domain.effects[o]) {
    int f = ToFact(domain.fact_offset, v);
    graph->marked[0][f] = 1;
    graph->marked[1][f] = 1;
  }
  return o;
}

int ExtractCost(const Domain &domain, const GraphSchema &schema,
                PlanningGraph *graph) {
  int m = graph->n_layers - 1;
  int h = 0;
  for (int i=m; i>0; --i) {
    std::fill(graph->marked[0].begin(), graph->marked[0].end(), 0);
    std::fill(graph->marked[1].begin(), graph->marked[1].end(), 0);
    for (auto g : graph->g_set[i]) {
      if (graph->marked[1][g] == 1) continue;
      int o = ExtractAction(i, g, domain, schema, graph);
      h += domain.costs[o];
    }
  }
  return h;
}

vector<int> ExtractPlan(const Domain &domain, const GraphSchema &schema,
                        PlanningGraph *graph) {
  int m = graph->n_layers - 1;
  vector< vector<int> > tmp(m);
  for (int i=m; i>0; --i) {
    std::fill(graph->marked[0].begin(), graph->marked[0].end(), 0);
    std::fill(graph->marked[1].begin(), graph->marked[1].end(), 0);
    for (auto g : graph->g_set[i]) {
      if (graph->marked[1][g] == 1) continue;
      int o = ExtractAction(i, g, domain, schema, graph);
      tmp[i-1].push_back(o);
    }
  }
  vector<int> result;
  for (int i=0; i<m; ++i)
    result.insert(result.end(), tmp[i].begin(), tmp[i].end());
  return result;
}

void ExtractPreferred(const GraphSchema &schema, const PlanningGraph &graph,
                      unordered_set<int> &preferred) {
  if (graph.n_layers < 2) return;
  for (auto g : graph.g_set[1]) {
    for (auto o : schema.effect_map[g])
      if (graph.action_layer_membership[o] == 0) preferred.insert(o);
  }
}

vector<int> Graphplan(const State &initial, const Domain &domain,
                      const GraphSchema &schema, PlanningGraph *graph,
                      unordered_set<int> &preferred) {
  preferred.clear();
  ConstructGraph(initial, domain, schema, graph);
  if (graph->n_layers == -1) return vector<int>{-1};
  InitializeGSet(schema, graph);
  auto result = ExtractPlan(domain, schema, graph);
  ExtractPreferred(schema, *graph, preferred);
  return result;
}

int GraphplanCost(const State &initial, const Domain &domain,
                  const GraphSchema &schema, PlanningGraph *graph) {
  ConstructGraph(initial, domain, schema, graph);
  if (graph->n_layers == -1) return std::numeric_limits<int>::max();
  InitializeGSet(schema, graph);

  return ExtractCost(domain, schema, graph);
}

int GraphplanCost(const State &initial, const Domain &domain,
                  const GraphSchema &schema, PlanningGraph *graph,
                  unordered_set<int> &preferred) {
  preferred.clear();
  ConstructGraph(initial, domain, schema, graph);
  if (graph->n_layers == -1) return std::numeric_limits<int>::max();
  InitializeGSet(schema, graph);
  int h = ExtractCost(domain, schema, graph);
  ExtractPreferred(schema, *graph, preferred);
  return h;
}

} // namespace graphplan
