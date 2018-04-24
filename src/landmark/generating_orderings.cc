#include "landmark/generating_orderings.h"

#include <iostream>
#include <utility>
#include <vector>

using std::make_pair;
using std::pair;
using std::vector;

namespace rwls {

bool AreInConsistent(const Domain &domain, VarValue l, VarValue l_p) {
  int var, value, var_p, value_p;
  DecodeVarValue(l, &var, &value);
  DecodeVarValue(l_p, &var_p, &value_p);
  if (var == var_p && value != value_p) return true;
  for (auto &mutex_group : domain.mutex_groups) {
    if (mutex_group.find(l) != mutex_group.end()
        && mutex_group.find(l_p) != mutex_group.end()) return true;
  }
  return false;
}

bool Interfere(const Domain &domain, const GraphSchema &schema,
               const LandmarkGraph &graph, const Landmark &l,
               const Landmark &l_p) {
  VarValue l_var_value = l.GetVarValue();
  VarValue l_p_var_value = l_p.GetVarValue();
  if (AreInConsistent(domain, l_var_value, l_p_var_value)) return true;

  std::unordered_map<VarValue, size_t> counter;
  bool initial = true;
  const vector<int> &achievers = graph.GetPossibleAchievers(graph.ToId(l));
  for (auto o : achievers) {
    for (auto x : domain.effects[o]) {
      if (initial)
        counter[x] = 1;
      else if (counter.find(x)  != counter.end())
        ++counter[x];
    }
  }
  size_t n_effects = achievers.size();
  for (auto v : counter) {
    if (v.second < n_effects) continue;
    if (AreInConsistent(domain, v.first, l_p_var_value)) return true;
  }

  // Fast Downward issue 202
  /**
  size_t l_id = graph.ToId(l);
  size_t l_p_id = graph.ToId(l_p);
  for (auto init_id : graph.GetInitIdsByTermId(l_id)) {
    if (init_id == l_p_id
        || graph.GetOrderingType(init_id, l_id) != LandmarkGraph::GREEDY)
      continue;
    VarValue x = graph.GetLandmark(init_id).GetVarValue();
    if (AreInConsistent(domain, x, l_p_var_value)) return true;
  }
   **/

  return false;
}

void AddByPath(const Domain &domain, const GraphSchema &schema,
               LandmarkGraph& graph,
               vector< pair<size_t, size_t> > &orderings) {
  vector<size_t> ancestors;
  size_t landmarks_size = graph.GetLandmarksSize();
  for (size_t l_p_id=domain.goal.size(); l_p_id<landmarks_size; ++l_p_id) {
    const Landmark &l_p = graph.GetLandmark(l_p_id);
    if (!l_p.IsFact() || l_p.IsImplicated(domain.initial)) return;
    // L_n+1
    for (auto l_n1_id : graph.GetTermIdsByInitId(l_p_id)) {
      if (graph.GetOrderingType(l_p_id, l_n1_id) != LandmarkGraph::GREEDY)
        continue;
      // L_n
      for (auto l_n_id : graph.GetInitIdsByTermId(l_n1_id)) {
        graph.GetAncestors(l_n_id, ancestors);
        // L
        for (auto l_id : ancestors) {
          if (l_id == l_p_id) continue;
          const Landmark &l = graph.GetLandmark(l_id);
          if (l.IsFact() && Interfere(domain, schema, graph, l, l_p))
            orderings.push_back(make_pair(l_id, l_p_id));
        }
      }
    }
  }
}

void ReasonableOrderings(const Domain &domain, const GraphSchema &schema,
                         LandmarkGraph *graph) {
  size_t goal_size = domain.goal.size();
  size_t landmarks_size = graph->GetLandmarksSize();
  vector< pair<size_t, size_t> > orderings;
  // L'
  for (size_t l_p_id=0; l_p_id<goal_size; ++l_p_id) {
    const Landmark &l_p = graph->GetLandmark(l_p_id);
    if (l_p.IsImplicated(domain.initial)) continue;
    // L
    for (size_t l_id=0; l_id<landmarks_size; ++l_id) {
      if (l_id == l_p_id) continue;
      const Landmark &l = graph->GetLandmark(l_id);
      if (l.IsFact() && Interfere(domain, schema, *graph, l, l_p))
        orderings.push_back(make_pair(l_id, l_p_id));
    }
  }

  AddByPath(domain, schema, *graph, orderings);
  for (auto ordering : orderings) {
    size_t init_id = ordering.first;
    size_t term_id = ordering.second;
    if (graph->IsAdjacent(term_id, init_id)) continue;
    graph->AddOrdering(init_id, term_id, LandmarkGraph::REASONABLE);
  }
}

void ObedientOrderings(const Domain &domain, const GraphSchema &schema,
                       LandmarkGraph *graph) {
  vector< pair<size_t, size_t> > orderings;
  AddByPath(domain, schema, *graph, orderings);
  for (auto ordering : orderings) {
    size_t init_id = ordering.first;
    size_t term_id = ordering.second;
    if (graph->IsAdjacent(term_id, init_id)) continue;
    graph->AddOrdering(init_id, term_id, LandmarkGraph::OBEDIENT);
  }
}

void AddOrderings(const Domain &domain, const GraphSchema &schema,
                  LandmarkGraph *graph) {
  ReasonableOrderings(domain, schema, graph);
  ObedientOrderings(domain, schema, graph);
}

void HandleCycles(LandmarkGraph *graph) {
  int n_edge = 0;
  for (size_t i=0, n=graph->GetLandmarks().size(); i<n; ++i) {
    if (graph->GetLandmark(i).IsEmpty()) continue;
    n_edge += graph->RemoveCycles(i);
  }
  std::cout << "Removed " << n_edge << " reasonable or obedient orders"
            << std::endl;
  std::cout << graph->GetOrderingsSize() << " edges" << std::endl;
}

} // namespace rwls
