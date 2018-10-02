#include "lts.h"

namespace pplanner {

using std::shared_ptr;
using std::vector;

constexpr int AtomicLTS::kInfinity;

int AtomicLTS::ShortestPathCost(int from, int to, bool only_tau) {
  if (only_tau && h_tau_cache_[from][to] != -1)
    return h_tau_cache_[from][to];

  if (!only_tau && h_star_cache_[from][to] != -1)
    return h_star_cache_[from][to];

  std::fill(closed_.begin(), closed_.end(), kInfinity);
  open_ = PQueue();
  open_.push(std::make_pair(0, from));

  while (!open_.empty()) {
    auto top = open_.top();
    open_.pop();

    int g = top.first;
    int s = top.second;

    if (closed_[s] < g) continue;

    if (s == to) {
      if (only_tau)
        h_tau_cache_[from][to] = g;
      else
        h_star_cache_[from][to] = g;

      return g;
    }

    for (auto l : labels_[s]) {
      if (only_tau && !IsTauLabel(l)) continue;

      int t = LabelTo(l);

      // ignore self loop
      if (t == -1) continue;

      int cost = g + LabelCost(l);

      if (closed_[t] == kInfinity || cost < closed_[t]) {
        closed_[t] = cost;
        open_.push(std::make_pair(cost, t));
      }
    }
  }

  if (only_tau)
    h_tau_cache_[from][to] = kInfinity;
  else
    h_star_cache_[from][to] = kInfinity;

  return kInfinity;
}

void AtomicLTS::InitTauCost(const vector<bool> &is_tau_label) {
  for (int i=0, n=is_tau_label.size(); i<n; ++i)
    if (is_tau_label[i]) tau_cost_[i] = 1;
}

void AtomicLTS::ClearTauCache() {
  for (auto &v : h_tau_cache_)
    std::fill(v.begin(), v.end(), -1);
}

void MakeState(int i, shared_ptr<const SASPlus> problem, vector<int> &precondition,
               vector<int> &effect) {
  std::fill(precondition.begin(), precondition.end(), -1);
  std::fill(effect.begin(), effect.end(), -1);

  auto var_itr = problem->PreconditionVarsBegin(i);
  auto value_itr = problem->PreconditionValuesBegin(i);
  auto end = problem->PreconditionVarsEnd(i);

  while (var_itr != end) {
    precondition[*var_itr] = *value_itr;
    ++var_itr;
    ++value_itr;
  }

  var_itr = problem->EffectVarsBegin(i);
  value_itr = problem->EffectValuesBegin(i);
  end = problem->EffectVarsEnd(i);

  while (var_itr != end) {
    effect[*var_itr] = *value_itr;
    ++var_itr;
    ++value_itr;
  }
}

int TauVariable(const vector<int> &precondition, const vector<int> &effect) {
  int tau_v = -1;

  for (int j=0, n=precondition.size(); j<n; ++j) {
    if (precondition[j] == -1 && effect[j] == -1) continue;

    if (tau_v == -1)
      tau_v = j;
    else
      return -1;
  }

  return tau_v;
}

vector<shared_ptr<AtomicLTS> > InitializeLTSs(shared_ptr<const SASPlus> problem)
{
  int n_variables = problem->n_variables();
  int n_actions = problem->n_actions();
  vector<vector<int> > label_from(n_variables,
                                vector<int>(n_actions, -1));
  vector<vector<int> > label_to(n_variables,
                                vector<int>(n_actions, -1));
  vector<vector<bool> > is_tau_label(n_variables,
                                     vector<bool>(n_actions, false));
  vector<vector<vector<int> > > labels(n_variables);

  for (int i=0; i<n_variables; ++i)
    labels[i].resize(problem->VarRange(i), vector<int>{-1});

  vector<int> precondition(n_variables);
  vector<int> effect(n_variables);

  for (int i=0, n=problem->n_actions(); i<n; ++i) {
    MakeState(i, problem, precondition, effect);
    int tau_v = TauVariable(precondition, effect);
    is_tau_label[tau_v][i] = true;

    for (int j=0; j<n_variables; ++j) {
      int precondition_value = precondition[j];
      label_from[j][i] = precondition_value;
      label_to[j][i] = effect[j];

      if (precondition_value == -1) {
        for (int k=0; k<problem->VarRange(j); ++k)
          labels[j][k].push_back(i);
      } else {
        labels[j][precondition_value].push_back(i);
      }
    }
  }

  auto initial = problem->initial();
  std::vector<int> goal(n_variables, -1);

  for (int i=0, n=problem->n_goal_facts(); i<n; ++i)
    goal[problem->GoalVar(i)] = problem->GoalValue(i);

  vector<shared_ptr<AtomicLTS> > ltss(n_variables, nullptr);

  for (int i=0; i<n_variables; ++i) {
    ltss[i] = std::make_shared<AtomicLTS>(initial[i], goal[i], label_from[i],
                                          label_to[i], is_tau_label[i],
                                          labels[i]);
  }

  return ltss;
}

int RecursiveTauPathCost(int i, int s_i, int t_i,
                         vector<shared_ptr<AtomicLTS> > &ltss) {
  auto &kInfinity = AtomicLTS::kInfinity;

  int h_max = 0;

  for (int j=0, n=ltss.size(); j<n; ++j) {
    if (j == i) continue;

    int t_j = ltss[j]->LabelFrom(j);

    if (t_j == -1) continue;

    for (int s_j=0, m=ltss[j]->n_states(); s_j<m; ++s_j) {
      if (s_j == t_j) continue;

      int h_s_t = ltss[j]->ShortestPathCost(s_j, t_j, true);

      if (h_s_t == kInfinity) return kInfinity;

      int h_t_s = ltss[j]->ShortestPathCost(t_j, s_j, true);

      if (h_t_s == kInfinity) return kInfinity;

      int h = h_s_t + h_t_s;

      if (h > h_max) h_max = h;
    }
  }

  return h_max;
}

bool AddRecursiveTauLabel(int l, vector<shared_ptr<AtomicLTS> > &ltss) {
  auto &kInfinity = AtomicLTS::kInfinity;

  int i = -1;

  for (int j=0, n=ltss.size(); j<n; ++j) {
    if (ltss[j]->IsTauLabel(l) || (ltss[j]->LabelTo(l) != -1 && i != -1))
      return kInfinity;

    if (ltss[j]->LabelTo(l) != -1) i = j;
  }

  int s_i = ltss[i]->LabelFrom(i);
  int t_i = ltss[i]->LabelTo(i);

  if (s_i == -1) {
    int h_max = 0;

    for (s_i=0; s_i<ltss[i]->n_states(); ++s_i) {
      if (s_i == t_i) continue;
      h_max = std::max(h_max, RecursiveTauPathCost(i, s_i, t_i, ltss));
    }

    if (h_max != kInfinity) {
      ltss[i]->SetTauLabel(l, ltss[i]->LabelCost(l) + h_max);

      return true;
    }
  } else {
    int h = RecursiveTauPathCost(i, s_i, t_i, ltss);

    if (h != kInfinity) {
      ltss[i]->SetTauLabel(l, ltss[i]->LabelCost(l) + h);

      return true;
    }
  }

  return false;
}

void AddRecursiveTauLabels(vector<shared_ptr<AtomicLTS> > &ltss) {
  bool condition = true;

  while (condition) {
    bool condition = false;

    for (int l=0, n=ltss[0]->n_labels(); l<n; ++l)
      condition = AddRecursiveTauLabel(l, ltss) || condition;
  }
}

} // namespace pplanner
