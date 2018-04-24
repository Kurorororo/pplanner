#include "heuristic/simplified_graphplan.h"

#include <cassert>

#include <limits>

#include "heuristic/graphplan.h"

using std::unordered_set;
using std::vector;

namespace rwls {

void InitializeGraph(const RelaxedDomain &domain, PlanningGraph *graph) {
  size_t n_facts = domain.fact_size;
  graph->fact_layer_membership.resize(n_facts);
  graph->closed.resize(n_facts);
  graph->marked[0].resize(n_facts);
  graph->marked[1].resize(n_facts);
  size_t n_actions = domain.action_size;
  graph->precondition_counter.resize(n_actions);
  graph->action_layer_membership.resize(n_actions);
}


bool FactLayer(const RelaxedDomain &domain, PlanningGraph *graph) {
  while (!graph->scheduled_facts.empty()) {
    int f = graph->scheduled_facts.back();
    graph->scheduled_facts.pop_back();
    graph->fact_layer_membership[f] = graph->n_layers;

    if (domain.is_goal[f] && ++graph->goal_counter == domain.goal_size)
      return true;

    for (auto o : domain.precondition_map[f]) {
      if (++graph->precondition_counter[o] == domain.precondition_size[o])
        graph->scheduled_actions.push_back(o);
    }
  }

  return false;
}

void ActionLayer(const RelaxedDomain &domain, PlanningGraph *graph)  {
  while (!graph->scheduled_actions.empty()) {
    int o = graph->scheduled_actions.back();
    graph->scheduled_actions.pop_back();
    graph->action_layer_membership[o] = graph->n_layers;

    int f = domain.effects[o];

    if (!graph->closed[f]) {
      graph->closed[f] = true;
      graph->scheduled_facts.push_back(f);
    }
  }
}

void ConstructGraph(const vector<int> &initial, const RelaxedDomain &domain,
                    PlanningGraph *graph) {
  ResetGraph(graph);

  for (auto f : initial) {
    graph->closed[f] = true;
    graph->scheduled_facts.push_back(f);
  }

  while (!graph->scheduled_facts.empty()) {
    bool is_end = FactLayer(domain, graph);

    if (is_end) {
      ++graph->n_layers;
      return;
    }

    ActionLayer(domain, graph);
    ++graph->n_layers;
  }

  graph->n_layers = -1;
}

void RistrictedFactLayer(const RelaxedDomain &domain,
                         const unordered_set<int> &black_list,
                         PlanningGraph *graph) {
  while (!graph->scheduled_facts.empty()) {
    int f = graph->scheduled_facts.back();
    graph->scheduled_facts.pop_back();
    graph->fact_layer_membership[f] = graph->n_layers;

    for (auto o : domain.precondition_map[f]) {
      if (++graph->precondition_counter[o] == domain.precondition_size[o]
          && black_list.find(domain.ids[o]) == black_list.end())
        graph->scheduled_actions.push_back(o);
    }
  }
}

void ConstructRRPG(const std::vector<int> &initial, const RelaxedDomain &domain,
                   const unordered_set<int> &black_list,
                   PlanningGraph *graph) {
  ResetGraph(graph);

  for (auto f : initial) {
    graph->closed[f] = 1;
    graph->scheduled_facts.push_back(f);
  }

  while (!graph->scheduled_facts.empty()) {
    RistrictedFactLayer(domain, black_list, graph);
    ActionLayer(domain, graph);
    ++graph->n_layers;
  }
}

int ChooseAction(int index, int i, const RelaxedDomain &domain,
                 const PlanningGraph &graph) {
  int min = -1;
  int argmin = 0;

  for (auto o : domain.effect_map[index]) {
    if (graph.action_layer_membership[o] != i-1) continue;
    int difficulty = 0;

    for (auto p : domain.preconditions[o])
      difficulty += graph.fact_layer_membership[p];

    if (difficulty < min || min == -1) {
      min = difficulty;
      argmin = o;
    }
  }

  assert(-1 != min);

  return argmin;
}

void InitializeGSet(const RelaxedDomain &domain, PlanningGraph *graph) {
  graph->g_set.resize(graph->n_layers);

  for (auto g : domain.goal)
    graph->g_set[graph->fact_layer_membership[g]].push_back(g);
}


int ExtractAction(int i, int g, const RelaxedDomain &domain,
                  PlanningGraph *graph) {
  int o = ChooseAction(g, i, domain, *graph);

  for (int f : domain.preconditions[o]) {
    int j = graph->fact_layer_membership[f];

    if (j != 0 && graph->marked[0][f] == 0)
      graph->g_set[j].push_back(f);
  }

  int f = domain.effects[o];
  graph->marked[0][f] = 1;
  graph->marked[1][f] = 1;

  return o;
}

int ExtractCost(const RelaxedDomain &domain, PlanningGraph *graph) {
  int m = graph->n_layers - 1;
  int h = 0;

  for (int i=m; i>0; --i) {
    std::fill(graph->marked[0].begin(), graph->marked[0].end(), 0);
    std::fill(graph->marked[1].begin(), graph->marked[1].end(), 0);

    for (auto g : graph->g_set[i]) {
      if (graph->marked[1][g] == 1) continue;
      int o = ExtractAction(i, g, domain, graph);
      h += domain.costs[o];
    }
  }

  return h;
}

vector<int> ExtractPlan(const RelaxedDomain &domain, PlanningGraph *graph) {
  int m = graph->n_layers - 1;
  vector< vector<int> > tmp(m);

  for (int i=m; i>0; --i) {
    std::fill(graph->marked[0].begin(), graph->marked[0].end(), 0);
    std::fill(graph->marked[1].begin(), graph->marked[1].end(), 0);

    for (auto g : graph->g_set[i]) {
      if (graph->marked[1][g] == 1) continue;
      int o = ExtractAction(i, g, domain, graph);
      tmp[i-1].push_back(o);
    }
  }

  vector<int> result;

  for (int i=0; i<m; ++i)
    for (auto a : tmp[i])
      result.push_back(domain.ids[a]);

  return result;
}

void ExtractPreferred(const RelaxedDomain &domain, const PlanningGraph &graph,
                      unordered_set<int> &preferred) {
  if (graph.n_layers < 2) return;

  for (auto g : graph.g_set[1]) {
    for (auto o : domain.effect_map[g])
      if (graph.action_layer_membership[o] == 0)
        preferred.insert(domain.ids[o]);
  }
}

vector<int> Graphplan(const vector<int> &initial, const RelaxedDomain &domain,
                      PlanningGraph *graph, unordered_set<int> &preferred) {
  preferred.clear();
  ConstructGraph(initial, domain, graph);
  if (graph->n_layers == -1) return vector<int>{-1};

  InitializeGSet(domain, graph);
  auto result = ExtractPlan(domain, graph);
  ExtractPreferred(domain, *graph, preferred);

  return result;
}

int GraphplanCost(const vector<int> &initial, const RelaxedDomain &domain,
                  PlanningGraph *graph) {
  ConstructGraph(initial, domain, graph);
  if (graph->n_layers == -1) return std::numeric_limits<int>::max();
  InitializeGSet(domain, graph);

  return ExtractCost(domain, graph);
}

int GraphplanCost(const vector<int> &initial, const RelaxedDomain &domain,
                  PlanningGraph *graph, unordered_set<int> &preferred) {
  preferred.clear();
  int h = GraphplanCost(initial, domain, graph);
  ExtractPreferred(domain, *graph, preferred);

  return h;
}

} // namespace graphplan
