#include "landmark/generating_orderings.h"

#include <algorithm>
#include <iostream>
#include <utility>
#include <unordered_map>
#include <vector>

namespace pplanner {

using std::make_pair;
using std::pair;
using std::shared_ptr;
using std::unordered_map;
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
  return IsInConsistent(problem, l.VarValue(0), r.VarValue(0));
}


bool Interfere(shared_ptr<const SASPlus> problem,
               shared_ptr<const LandmarkGraph> graph, const Landmark &l,
               const Landmark &l_p) {
  static unordered_map<pair<int, int>, int, PairHash<int, int> > counter;
  static vector<pair<int, int> > effect;

  if (IsInConsistent(problem, l, l_p)) return true;

  // I wonder why commenting out this improves performance on woodworking.
  /**
  const auto &achievers = graph->GetPossibleAchievers(graph->ToId(l));
  counter.clear();

  for (auto o : achievers) {
    problem->CopyEffect(o, effect);

    for (auto e : effect) {
      auto iter = counter.find(e);

      if (iter == counter.end())
        counter[e] = 1;
      else
        ++(iter->second);
    }
  }

  int n_effects = achievers.size();

  for (auto p : counter) {
    if (p.second == n_effects
        && IsInConsistent(problem, p.first, l_p.VarValue(0)))
      return true;
  }
   **/

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
  int landmarks_size = graph->n_landmarks();
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
  int landmarks_size = graph->n_landmarks();
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

  for (int i=0, n=graph->landmark_id_max(); i<n; ++i) {
    if (graph->GetLandmark(i).IsEmpty()) continue;
    n_edge += graph->RemoveCycles(i);
  }

  std::cout << "Removed " << n_edge << " reasonable or obedient orders"
            << std::endl;
  std::cout << graph->n_orderings() << " edges" << std::endl;
}

} // namespace pplanner
