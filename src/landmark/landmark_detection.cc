#include "landmark_detection.h"

#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "domain/var_value.h"
#include "landmark/dtg.h"
#include "landmark/landmark.h"

using std::queue;
using std::string;
using std::unordered_map;
using std::unordered_set;
using std::vector;

int n_disj = 0;
int n_initial = 0;

namespace rwls {

void SetPossibleAchievers(const Landmark &psi, const Domain &domain,
                          const GraphSchema &schema, LandmarkGraph *graph) {
  size_t psi_id = graph->ToId(psi);
  for (auto v : psi.GetVarValues()) {
    int f = ToFact(domain.fact_offset, v);
    for (auto o : schema.effect_map[f])
      graph->PushPossibleAchiever(psi_id, o);
  }
}

void RRPG(const vector<int> &possible_achievers, const Domain &domain,
          const GraphSchema &schema, unordered_set<int> &black_list,
          PlanningGraph *rrpg) {
  black_list.clear();
  for (auto o : possible_achievers)
    black_list.insert(o);
  ConstructRRPG(domain.initial, domain, schema, black_list, rrpg);
}

void SetFirstAchievers(const Landmark &psi, const GraphSchema &schema,
                       const PlanningGraph &rrpg, LandmarkGraph *graph) {
  size_t psi_id = graph->ToId(psi);
  for (auto o : graph->GetPossibleAchievers(psi_id))
    if (rrpg.precondition_counter[o] == schema.precondition_size[o])
      graph->PushFirstAchiever(psi_id, o);
}

vector<VarValue> ExtendedPreconditions(const Landmark &psi,
                                       const Domain &domain, int id) {
  vector<bool> has_precondition(domain.variables_size, false);
  for (auto p : domain.preconditions[id]) {
    int var = GetVar(p);
    has_precondition[var] = true;
  }

  vector<VarValue> preconditions = domain.preconditions[id];
  for (auto e : domain.effects[id]) {
    int var = GetVar(e);
    if (!has_precondition[var] && domain.dom[var] == 2) {
      for (auto psi_v : psi.GetVarValues()) {
        int psi_var, psi_value;
        DecodeVarValue(psi_v, &psi_var, &psi_value);
        if (psi_var == var && psi_value != domain.initial[var]) {
          VarValue v;
          EncodeVarValue(var, domain.initial[var], &v);
          preconditions.push_back(v);
        }
      }
    }
  }
  return preconditions;
}

void PreShared(const Landmark &psi, const Domain &domain,
               const LandmarkGraph &graph,
               unordered_map<VarValue, size_t> &pre_shared) {

  pre_shared.clear();
  size_t psi_id = graph.ToId(psi);
  auto &achievers = graph.GetFirstAchievers(psi_id);
  if (achievers.empty()) return;
  for (auto p : ExtendedPreconditions(psi, domain, achievers[0]))
    pre_shared[p] = 1;

  size_t n = achievers.size();
  for (size_t i=1; i<n; ++i)
    for (auto p : ExtendedPreconditions(psi, domain, achievers[i]))
      if (pre_shared.find(p) != pre_shared.end()) ++pre_shared[p];

  auto it = pre_shared.begin();
  while (it != pre_shared.end()) {
    if (it->second < n)
      it = pre_shared.erase(it);
    else
      ++it;
  }
}

void PreDisj(const Landmark &psi, const Domain &domain,
             const LandmarkGraph &graph,
             unordered_map<string, size_t> &candidate_counts,
             unordered_map<string, Landmark> &pre_disj) {
  candidate_counts.clear();
  pre_disj.clear();
  size_t psi_id = graph.ToId(psi);
  auto &achievers = graph.GetFirstAchievers(psi_id);
  if (achievers.empty()) return;
  size_t i = 0;
  for (auto o : achievers) {
    for (auto p : ExtendedPreconditions(psi, domain, o)) {
      int f = ToFact(domain.fact_offset, p);
      string predicate = domain.fact_to_predicate[f];
      if (i == 0 || candidate_counts[predicate] == i) {
        pre_disj[predicate].AddVarValue(p);
        candidate_counts[predicate] = i + 1;
      }
    }
    ++i;
  }

  size_t n = achievers.size();
  for (auto c : candidate_counts) {
    const Landmark &l = pre_disj[c.first];
    if (c.second < n || l.IsFact() || l.GetSize() > 4) pre_disj.erase(c.first);
  }
}

void PreLookAhead(const Domain &domain, const PlanningGraph &rrpg, int var,
                  int start, int goal, DTG *dtg, vector<int> &pre_lookahead) {
  pre_lookahead.clear();
  dtg->RemoveNodesByRRPG(domain, rrpg, var, goal);
  for (int i=0, n=domain.dom[var]; i<n; ++i) {
    if (i == goal) continue;
    if (!dtg->IsConnected(start, goal, i)) pre_lookahead.push_back(i);
  }
}

void AddFactLandmark(const Landmark &phi, LandmarkGraph *graph) {
  if (graph->IsIn(phi)) return;
  int var, value;
  DecodeVarValue(phi.GetVarValue(), &var, &value);
  for (size_t i=0, n=graph->GetLandmarksSize(); i<n; ++i) {
    const Landmark &chi = graph->GetLandmark(i);
    if (chi.IsEmpty() || phi == chi || !chi.IsImplicated(var, value)) continue;
    --n_disj;
    graph->Delete(i);
  }
}

void AddLandmarkAndOrdering(Landmark &phi, size_t term_id, int ordering_type,
                            queue<size_t> &q, LandmarkGraph *graph) {
  if (phi.IsFact())
    AddFactLandmark(phi, graph);

  for (auto &chi : graph->GetLandmarks())
    for (auto v : chi.GetVarValues())
      for (auto w : phi.GetVarValues())
        if (chi != phi && v == w) return;

  size_t phi_id;
  if (graph->IsIn(phi)) {
    phi_id = graph->ToId(phi);
  } else {
    if (phi.GetSize() > 1) ++n_disj;
    phi_id = graph->Add(phi);
    q.push(phi_id);
  }

  graph->AddOrdering(phi_id, term_id, ordering_type);
}

bool HaveSameOperator(const Landmark &psi, const Domain &domain,
                      const GraphSchema &schema, int f) {
  for (auto v : psi.GetVarValues()) {
    int f_v = ToFact(domain.fact_offset, v);
    for (auto o_v : schema.effect_map[f_v])
      for (auto o_f : schema.effect_map[f])
        if (o_v == o_f) return true;
  }
  return false;
}

void ExtendPotential(
    const Landmark &psi,
    const Domain &domain,
    const GraphSchema &schema,
    const PlanningGraph &rrpg,
    const LandmarkGraph &graph,
    unordered_map< size_t, unordered_set<VarValue> > &potential_orderings) {
  size_t psi_id = graph.ToId(psi);
  int i = 0;
  for (auto d : domain.dom) {
    for (int j=0; j<d; ++j) {
      int f = ToFact(domain.fact_offset, i, j);
      if (rrpg.fact_layer_membership[f] != -1) continue;
      VarValue i_j;
      EncodeVarValue(i, j, &i_j);
      if (HaveSameOperator(psi, domain, schema, f)) continue;
      potential_orderings[psi_id].insert(i_j);
    }
    ++i;
  }
}

void AddFurtherOrderings(
    const unordered_map<size_t, unordered_set<VarValue> > &potential_orderings,
    LandmarkGraph *graph) {
  for (auto &orderings : potential_orderings) {
    size_t init_id = orderings.first;
    if (graph->GetLandmark(init_id).IsEmpty()) continue;
    for (auto term_var_value : orderings.second) {
      Landmark psi(term_var_value);
      if (!graph->IsIn(psi)) continue;
      size_t term_id = graph->ToId(psi);
      if (init_id == term_id) continue;
      graph->AddOrdering(init_id, term_id, LandmarkGraph::NATURAL);
    }
  }
}

void IdentifyLandmarks(const Domain &domain, const GraphSchema &schema,
                       LandmarkGraph *graph) {
  const State &initial = domain.initial;
  queue<size_t> q;
  for (auto g : domain.goal) {
    Landmark phi(g);
    size_t id = graph->Add(phi);
    q.push(id);
  }

  unordered_map< size_t, unordered_set<VarValue> > potential_orderings;

  PlanningGraph rrpg;
  InitializeGraph(domain, schema, &rrpg);
  unordered_set<int> black_list;

  auto dtgs = DTG::InitializeDTGs(domain);

  unordered_map<VarValue, size_t> pre_shared;
  unordered_map<string, size_t> candidate_counts;
  unordered_map<string, Landmark> pre_disj;
  vector<int> pre_lookahead;

  while (!q.empty()) {
    size_t psi_id = q.front();
    q.pop();
    Landmark psi = graph->CopyLandmark(psi_id);
    if (psi.IsEmpty()) continue;
    SetPossibleAchievers(psi, domain, schema, graph);
    if (psi.IsImplicated(initial)) {
      ++n_initial;
      continue;
    }

    auto &p_achievers = graph->GetPossibleAchievers(psi_id);
    RRPG(p_achievers, domain, schema, black_list, &rrpg);
    SetFirstAchievers(psi, schema, rrpg, graph);
    PreShared(psi, domain, *graph, pre_shared);
    for (auto p : pre_shared) {
      Landmark phi(p.first);
      AddLandmarkAndOrdering(phi, psi_id, LandmarkGraph::GREEDY, q, graph);
    }

    PreDisj(psi, domain, *graph, candidate_counts, pre_disj);
    for (auto v : pre_disj) {
      Landmark &phi = v.second;
      if (phi.IsImplicated(initial)) continue;
      AddLandmarkAndOrdering(phi, psi_id, LandmarkGraph::GREEDY, q, graph);
    }
    if (psi.IsFact()) {
      int var, goal;
      DecodeVarValue(psi.GetVarValue(), &var, &goal);
      DTG &dtg = dtgs[var];
      PreLookAhead(domain, rrpg, var, initial[var], goal, &dtg, pre_lookahead);
      for (auto value : pre_lookahead) {
        Landmark phi(var, value);
        AddLandmarkAndOrdering(phi, psi_id, LandmarkGraph::NATURAL, q, graph);
      }
    }
    ExtendPotential(psi, domain, schema, rrpg, *graph, potential_orderings);
  }
  AddFurtherOrderings(potential_orderings, graph);

  std::cout << "Discovered " << graph->GetLandmarksSize()
            << " landmarks, of which " << n_disj << " are disjunctive"
            << std::endl;
  std::cout << n_initial << " initial landmarks, " << domain.goal.size()
            << " goal landmarks"  << std::endl;
}

} // namespace rwls
