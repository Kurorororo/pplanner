#include "landmark_detection.h"

#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "dtg.h"
#include "heuristics/rpg.h"
#include "landmark/landmark.h"

namespace pplanner {

using std::make_pair;
using std::pair;
using std::queue;
using std::shared_ptr;
using std::string;
using std::unordered_map;
using std::unordered_set;
using std::vector;

using pair_map = unordered_map<pair<int, int>, int, PairHash<int, int> >;
using pair_set = unordered_set<pair<int, int>, PairHash<int, int> >;

int n_disj = 0;
int n_initial = 0;

void SetPossibleAchievers(const Landmark &psi,
                          shared_ptr<const SASPlus> problem,
                          shared_ptr<const RelaxedSASPlus> r_problem,
                          shared_ptr<LandmarkGraph> graph) {
  static vector<bool> closed(problem->n_actions(), false);

  std::fill(closed.begin(), closed.end(), false);
  int psi_id = graph->ToId(psi);

  for (int i=0, n=psi.size(); i<n; ++i) {
    int f = problem->Fact(psi.GetVarValue(i));

    for (auto o : r_problem->EffectMap(f)) {
      int a = r_problem->ActionId(o);

      if (!closed[a]) {
        graph->PushPossibleAchiever(psi_id, a);
        closed[a] = true;
      }
    }
  }
}

void RRPG(const vector<int> &possible_achievers,
          const vector<int> &initial,
          vector<bool> &black_list,
          RPG *rrpg) {
  std::fill(black_list.begin(), black_list.end(), false);

  for (auto o : possible_achievers)
    black_list[o] = true;

  rrpg->ConstructRRPG(initial, black_list);
}

void SetFirstAchievers(const Landmark &psi, const RPG &rrpg,
                       shared_ptr<LandmarkGraph> graph) {
  int psi_id = graph->ToId(psi);

  for (auto o : graph->GetPossibleAchievers(psi_id))
    if (rrpg.IsInAction(o)) graph->PushFirstAchiever(psi_id, o);
}

vector<pair<int, int> > ExtendedPreconditions(const Landmark &psi,
                                              shared_ptr<const SASPlus> problem,
                                              int action) {
  static vector<bool> has_precondition(problem->n_variables(), false);

  std::fill(has_precondition.begin(), has_precondition.end(), false);
  vector<pair<int, int> > precondition;
  problem->CopyPrecondition(action, precondition);

  for (auto p : precondition)
    has_precondition[p.first] = true;

  auto iter = problem->EffectVarsBegin(action);
  auto end = problem->EffectVarsEnd(action);
  auto initial = problem->initial();

  for (; iter != end; ++iter) {
    int var = *iter;

    if (!has_precondition[var] && problem->VarRange(var) == 2)
      for (int i=0, n=psi.size(); i<n; ++i)
        if (psi.GetVar(i) == var && psi.GetValue(i) != initial[var])
          precondition.push_back(make_pair(var, initial[var]));
  }

  return precondition;
}

const pair_map& PreShared(const Landmark &psi,
                          shared_ptr<const SASPlus> problem,
                          shared_ptr<const LandmarkGraph> graph) {
  static pair_map pre_shared;

  pre_shared.clear();
  int psi_id = graph->ToId(psi);
  auto &achievers = graph->GetFirstAchievers(psi_id);
  if (achievers.empty()) return pre_shared;

  for (auto p : ExtendedPreconditions(psi, problem, achievers[0]))
    pre_shared[p] = 1;

  int n = achievers.size();

  for (int i=1; i<n; ++i)
    for (auto p : ExtendedPreconditions(psi, problem, achievers[i]))
      if (pre_shared.find(p) != pre_shared.end()) ++pre_shared[p];

  auto it = pre_shared.begin();

  while (it != pre_shared.end()) {
    if (it->second < n)
      it = pre_shared.erase(it);
    else
      ++it;
  }

  return pre_shared;
}

const unordered_map<string, Landmark>& PreDisj(
    const Landmark &psi,
    shared_ptr<const SASPlus> problem,
    shared_ptr<const LandmarkGraph> graph,
    unordered_map<string, int> &candidate_counts) {
  static unordered_map<string, Landmark> pre_disj;

  candidate_counts.clear();
  pre_disj.clear();

  int psi_id = graph->ToId(psi);
  auto &achievers = graph->GetFirstAchievers(psi_id);
  if (achievers.empty()) return pre_disj;
  int i = 0;

  for (auto o : achievers) {
    for (auto p : ExtendedPreconditions(psi, problem, o)) {
      string predicate = problem->Predicate(p.first, p.second);

      if (i == 0 || candidate_counts[predicate] == i) {
        pre_disj[predicate].Add(p);
        candidate_counts[predicate] = i + 1;
      }
    }

    ++i;
  }

  int n = achievers.size();

  for (auto c : candidate_counts) {
    const Landmark &l = pre_disj[c.first];

    if (c.second < n || l.IsFact() == 1 || l.size() > 4)
      pre_disj.erase(c.first);
  }

  return pre_disj;
}

void RemoveNodesByRRPG(shared_ptr<const SASPlus> &problem, const RPG &rrpg,
                       int var, int goal_value, DTG *dtg) {
  dtg->RecoverSoftDelete();

  for (int i=0, m=problem->VarRange(var); i<m; ++i) {
    if (i == goal_value) continue;
    int f = problem->Fact(var, i);
    if (!rrpg.IsInFact(f)) dtg->SoftRemoveNode(i);
  }
}

void PreLookAhead(shared_ptr<const SASPlus> &problem, const RPG &rrpg,
                  int var, int start, int goal, DTG *dtg,
                  vector<int> &pre_lookahead) {
  pre_lookahead.clear();
  RemoveNodesByRRPG(problem, rrpg, var, goal, dtg);

  for (int i=0, n=problem->VarRange(var); i<n; ++i) {
    if (i == goal) continue;
    if (!dtg->IsConnected(start, goal, i)) pre_lookahead.push_back(i);
  }
}

void AddFactLandmark(const Landmark &phi, shared_ptr<LandmarkGraph> graph) {
  if (!phi.IsFact() || graph->IsIn(phi)) return;

  auto f = phi.GetVarValue(0);

  for (size_t i=0, n=graph->n_landmarks(); i<n; ++i) {
    const Landmark &chi = graph->GetLandmark(i);
    if (chi.IsEmpty() || phi == chi || !chi.IsImplicated(f)) continue;
    --n_disj;
    graph->Delete(i);
  }
}

void AddLandmarkAndOrdering(Landmark &phi, int term_id,
                            LandmarkGraph::OrderingType type, queue<int> &q,
                            shared_ptr<LandmarkGraph> graph) {
  AddFactLandmark(phi, graph);

  for (auto &chi : graph->GetLandmarks())
    if (phi.Overlap(chi)) return;

  int phi_id;

  if (graph->IsIn(phi)) {
    phi_id = graph->ToId(phi);
  } else {
    if (phi.size() > 1) ++n_disj;
    phi_id = graph->Add(phi);
    q.push(phi_id);
  }

  graph->AddOrdering(phi_id, term_id, type);
}

bool HaveSameOperator(const Landmark &psi, shared_ptr<const SASPlus> problem,
                      shared_ptr<const RelaxedSASPlus> r_problem, int f) {
  for (int i=0, n=psi.size(); i<n; ++i) {
    int f_v = problem->Fact(psi.GetVarValue(i));

    for (auto o_v : r_problem->EffectMap(f_v))
      for (auto o_f : r_problem->EffectMap(f))
        if (r_problem->ActionId(o_v) == r_problem->ActionId(o_f)) return true;
  }

  return false;
}

void ExtendPotential(
    const Landmark &psi,
    shared_ptr<const SASPlus> problem,
    shared_ptr<const RelaxedSASPlus> r_problem,
    const RPG &rrpg,
    shared_ptr<const LandmarkGraph> graph,
    unordered_map<int, pair_set> &potential_orderings) {
  int psi_id = graph->ToId(psi);

  for (int i=0, n=problem->n_variables(); i<n; ++i) {
    for (int j=0, m=problem->VarRange(i); j<m; ++j) {
      int f = problem->Fact(i, j);
      if (rrpg.IsInFact(f)) continue;
      if (HaveSameOperator(psi, problem, r_problem, f)) continue;
      potential_orderings[psi_id].insert(std::make_pair(i, j));
    }
  }
}

void AddFurtherOrderings(
    const unordered_map<int, pair_set> &potential_orderings,
    shared_ptr<LandmarkGraph> graph) {
  for (auto &orderings : potential_orderings) {
    int init_id = orderings.first;
    if (graph->GetLandmark(init_id).IsEmpty()) continue;

    for (auto term_var_value : orderings.second) {
      Landmark psi(term_var_value);
      if (!graph->IsIn(psi)) continue;
      int term_id = graph->ToId(psi);
      if (init_id == term_id) continue;
      graph->AddOrdering(init_id, term_id, LandmarkGraph::NATURAL);
    }
  }
}

void IdentifyLandmarks(shared_ptr<const SASPlus> problem,
                       shared_ptr<const RelaxedSASPlus> r_problem,
                       shared_ptr<LandmarkGraph> graph) {
  auto initial = problem->initial();
  vector<int> initial_facts;
  StateToFactVector(*problem, initial, initial_facts);
  queue<int> q;

  vector<pair<int, int> > goal;
  problem->CopyGoal(goal);

  for (auto g : goal) {
    Landmark phi(g);
    int id = graph->Add(phi);
    q.push(id);
  }

  unordered_map<int, pair_set> potential_orderings;

  RPG rrpg(r_problem);
  vector<bool> black_list(problem->n_actions(), false);

  auto dtgs = InitializeDTGs(problem);

  unordered_map<string, int> candidate_counts;
  vector<int> pre_lookahead;

  while (!q.empty()) {
    int psi_id = q.front();
    q.pop();
    Landmark psi = graph->CopyLandmark(psi_id);
    if (psi.IsEmpty()) continue;
    SetPossibleAchievers(psi, problem, r_problem, graph);

    if (psi.IsImplicated(initial)) {
      ++n_initial;
      continue;
    }

    auto &p_achievers = graph->GetPossibleAchievers(psi_id);
    RRPG(p_achievers, initial_facts, black_list, &rrpg);
    SetFirstAchievers(psi, rrpg, graph);
    auto &pre_shared = PreShared(psi, problem, graph);

    for (auto p : pre_shared) {
      Landmark phi(p.first);
      AddLandmarkAndOrdering(phi, psi_id, LandmarkGraph::GREEDY, q, graph);
    }

    auto &pre_disj = PreDisj(psi, problem, graph, candidate_counts);

    for (auto v : pre_disj) {
      Landmark &phi = v.second;
      if (phi.IsImplicated(initial)) continue;
      AddLandmarkAndOrdering(phi, psi_id, LandmarkGraph::GREEDY, q, graph);
    }

    if (psi.IsFact()) {
      int var = psi.GetVar(0);
      int goal = psi.GetValue(0);
      DTG &dtg = dtgs[var];
      PreLookAhead(problem, rrpg, var, initial[var], goal, &dtg, pre_lookahead);

      for (auto value : pre_lookahead) {
        Landmark phi(var, value);
        AddLandmarkAndOrdering(phi, psi_id, LandmarkGraph::NATURAL, q, graph);
      }
    }

    ExtendPotential(psi, problem, r_problem, rrpg, graph, potential_orderings);
  }

  AddFurtherOrderings(potential_orderings, graph);

  std::cout << "Discovered " << graph->n_landmarks()
            << " landmarks, of which " << n_disj << " are disjunctive"
            << std::endl;
  std::cout << n_initial << " initial landmarks, " << problem->n_goal_facts()
            << " goal landmarks"  << std::endl;
}

} // namespace pplanner
