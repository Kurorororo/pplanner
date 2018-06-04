#include "landmark/generating_orderings.h"

#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>

namespace pplanner {

using std::make_pair;
using std::pair;
using std::shared_ptr;
using std::vector;

inline bool IsInConsistent(shared_ptr<const SASPlus> problem,
                           const pair<int, int> &l, const pair<int, int> &r) {
  int l_var = l.first;
  int l_value = l.second;
  int r_var = r.first;
  int r_value = r.second;

  return (l_var == r_var && l_value != r_value)
    || problem->IsMutex(l_var, l_value, r_var, r_value);
}

inline bool IsInConsistent(shared_ptr<const SASPlus> problem,
                           const Landmark &l, const Landmark &r) {
  return IsInConsistent(problem, l.GetVarValue(0), r.GetVarValue(0));
}


bool Interfere(shared_ptr<const SASPlus> problem,
               shared_ptr<const LandmarkGraph> graph, const Landmark &l,
               const Landmark &l_p) {
  static vector<int> counter(problem->n_facts(), 0);

  if (IsInConsistent(problem, l, l_p)) return true;

  const auto &achievers = graph->GetPossibleAchievers(graph->ToId(l));
  std::fill(counter.begin(), counter.end(), 0);

  for (auto o : achievers) {
    auto var_iter = problem->EffectVarsBegin(o);
    auto value_iter = problem->EffectValuesBegin(o);

    for (auto end=problem->EffectVarsEnd(o); var_iter != end; ++var_iter) {
      ++counter[problem->Fact(*var_iter, *value_iter)];
      ++value_iter;
    }
  }

  int n_effects = achievers.size();
  int l_p_var = l_p.GetVar(0);
  int l_p_fact = problem->Fact(l_p.GetVarValue(0));
  int l_p_var_begin = problem->VarBegin(l_p_var);
  int l_p_var_end = l_p_var_begin + problem->VarRange(l_p_var);

  for (int i=0, n=counter.size(); i<n; ++i) {
    if (counter[i] < n_effects) continue;

    if ((l_p_var_begin <= i && i < l_p_var_end && i != l_p_fact)
        || problem->IsMutex(i, l_p_fact)) return true;
  }

  // Fast Downward issue 202
  /**
  int l_id = graph->ToId(l);
  int l_p_id = graph->ToId(l_p);
  for (auto init_id : graph->GetInitIdsByTermId(l_id)) {
    if (init_id == l_p_id
        || graph->GetOrderingType(init_id, l_id) != LandmarkGraph::GREEDY)
      continue;

    auto x = graph->GetLandmark(init_id);
    if (IsInConsistent(problem, x, l_p)) return true;
  }
   **/

  return false;
}

void AddByPath(shared_ptr<const SASPlus> problem,
               shared_ptr<const RelaxedSASPlus> r_problem,
               shared_ptr<const LandmarkGraph> graph,
               vector<pair<int, int> > &orderings) {
  vector<int> ancestors;
  int landmarks_size = graph->GetLandmarksSize();
  auto initial = problem->initial();

  for (int l_p_id=problem->n_goal_facts(); l_p_id<landmarks_size; ++l_p_id) {
    const Landmark &l_p = graph->GetLandmark(l_p_id);
    if (!l_p.IsFact() || l_p.IsImplicated(initial)) return;
    // L_n+1
    for (auto l_n1_id : graph->GetTermIdsByInitId(l_p_id)) {
      if (graph->GetOrderingType(l_p_id, l_n1_id) != LandmarkGraph::GREEDY)
        continue;
      // L_n
      for (auto l_n_id : graph->GetInitIdsByTermId(l_n1_id)) {
        graph->GetAncestors(l_n_id, ancestors);
        // L
        for (auto l_id : ancestors) {
          if (l_id == l_p_id) continue;
          const Landmark &l = graph->GetLandmark(l_id);

          if (l.IsFact() && Interfere(problem, graph, l, l_p))
            orderings.push_back(make_pair(l_id, l_p_id));
        }
      }
    }
  }
}

void ReasonableOrderings(shared_ptr<const SASPlus> problem,
                         shared_ptr<const RelaxedSASPlus> r_problem,
                         shared_ptr<LandmarkGraph> graph) {
  int goal_size = problem->n_goal_facts();
  int landmarks_size = graph->GetLandmarksSize();
  vector<pair<int, int> > orderings;
  auto initial = problem->initial();
  // L'
  for (int l_p_id=0; l_p_id<goal_size; ++l_p_id) {
    const Landmark &l_p = graph->GetLandmark(l_p_id);
    if (l_p.IsImplicated(initial)) continue;
    // L
    for (int l_id=0; l_id<landmarks_size; ++l_id) {
      if (l_id == l_p_id) continue;
      const Landmark &l = graph->GetLandmark(l_id);
      l.IsFact();
      Interfere(problem, graph, l, l_p);
      if (l.IsFact() && Interfere(problem, graph, l, l_p))
        orderings.push_back(make_pair(l_id, l_p_id));
    }
  }

  AddByPath(problem, r_problem, graph, orderings);

  for (auto ordering : orderings) {
    int init_id = ordering.first;
    int term_id = ordering.second;
    if (graph->IsAdjacent(term_id, init_id)) continue;
    graph->AddOrdering(init_id, term_id, LandmarkGraph::REASONABLE);
  }
}

void ObedientOrderings(shared_ptr<const SASPlus> problem,
                       shared_ptr<const RelaxedSASPlus> r_problem,
                       shared_ptr<LandmarkGraph> graph) {
  vector<pair<int, int> > orderings;
  AddByPath(problem, r_problem, graph, orderings);

  for (auto ordering : orderings) {
    int init_id = ordering.first;
    int term_id = ordering.second;
    if (graph->IsAdjacent(term_id, init_id)) continue;
    graph->AddOrdering(init_id, term_id, LandmarkGraph::OBEDIENT);
  }
}

void AddOrderings(shared_ptr<const SASPlus> problem,
                  shared_ptr<const RelaxedSASPlus> r_problem,
                  shared_ptr<LandmarkGraph> graph) {
  ReasonableOrderings(problem, r_problem, graph);
  ObedientOrderings(problem, r_problem, graph);
}

void HandleCycles(shared_ptr<LandmarkGraph> graph) {
  int n_edge = 0;

  for (int i=0, n=graph->GetLandmarks().size(); i<n; ++i) {
    if (graph->GetLandmark(i).IsEmpty()) continue;
    n_edge += graph->RemoveCycles(i);
  }

  std::cout << "Removed " << n_edge << " reasonable or obedient orders"
            << std::endl;
  std::cout << graph->GetOrderingsSize() << " edges" << std::endl;
}

} // namespace pplanner
