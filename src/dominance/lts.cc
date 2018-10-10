#include "lts.h"

#include <algorithm>
#include <iostream>

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

    for (auto t : To(s)) {
      // ignore self loop
      if (t == s) continue;

      int cost = g;

      if (only_tau) {
        int c = TauTransitionCost(s, t);
        if (c == kInfinity) continue;
        cost += c;
      } else {
        cost += TransitionCost(s, t);
      }

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
  int range = n_states();

  for (int s=0; s<range; ++s)
    for (int t=0; t<range; ++t)
      for (auto l : labels_[s][t])
        if (l != -1 && is_tau_label[l])
          tau_cost_[s][t] = 1;
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
  vector<vector<vector<vector<int> > > > labels(n_variables);
  vector<vector<vector<int> > > to(n_variables);

  for (int i=0; i<n_variables; ++i) {
    int range = problem->VarRange(i);
    to[i].resize(range);
    labels[i].resize(range, vector<vector<int> >(range));

    for (int v=0; v<range; ++v) {
      to[i][v].push_back(v);
      labels[i][v][v].push_back(-1);
    }
  }

  vector<int> precondition(n_variables);
  vector<int> effect(n_variables);

  for (int i=0, n=problem->n_actions(); i<n; ++i) {
    MakeState(i, problem, precondition, effect);
    int tau_v = TauVariable(precondition, effect);
    if (tau_v != -1) is_tau_label[tau_v][i] = true;

    for (int j=0; j<n_variables; ++j) {
      int precondition_value = precondition[j];
      int effect_value = effect[j];

      label_from[j][i] = precondition_value;
      label_to[j][i] = effect_value;

      if (precondition_value == -1) {
       /** Ignore always applicable labels other than noop for fast computation.
        * These labels are always dominated by themselfvs,
        * but possibly dominate other labels.
        * Thus ignoring these labels is safe, but may result in
        * weaker dominance relation/function.
        */
        continue;
        if (effect_value == -1) {
          for (int k=0; k<problem->VarRange(j); ++k)
            labels[j][k][k].push_back(i);
        } else {
          for (int k=0; k<problem->VarRange(j); ++k) {
            labels[j][k][effect_value].push_back(i);
            auto itr = std::find(to[j][k].begin(), to[j][k].end(),
                                 effect_value);

            if (itr == to[j][k].end())
              to[j][k].push_back(effect_value);
          }
        }
      } else {
        if (effect_value == -1) {
          labels[j][precondition_value][precondition_value].push_back(i);
        } else {
          labels[j][precondition_value][effect_value].push_back(i);
          auto itr = std::find(to[j][precondition_value].begin(),
                               to[j][precondition_value].end(), effect_value);

          if (itr == to[j][precondition_value].end())
            to[j][precondition_value].push_back(effect_value);
        }
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
                                          label_to[i], is_tau_label[i], to[i],
                                          labels[i]);
  }

  //AddRecursiveTauLabels(ltss);

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

void AtomicLTS::Dump() const {
  int n = n_states();

  for (int s=0; s<n; ++s) {
    if (s == initial())
      std::cout << "(initial) ";

    if (s == goal())
      std::cout << "(goal) ";

    std::cout << s << " -> ";

    for (auto t : To(s)) {
      for (auto l : Labels(s, t)) {
        std::cout << t << "(" << l;

        if (IsTauLabel(l))
          std::cout << " tau";

        std::cout << ") ";
      }
    }

    std::cout << std::endl;
  }
}

bool AddRecursiveTauLabel(int l, vector<shared_ptr<AtomicLTS> > &ltss) {
  auto &kInfinity = AtomicLTS::kInfinity;

  int i = -1;

  for (int j=0, n=ltss.size(); j<n; ++j) {
    if (ltss[j]->IsTauLabel(l) || (ltss[j]->LabelTo(l) != -1 && i != -1))
      return kInfinity;

    if (ltss[j]->LabelTo(l) != -1) i = j;
  }

  int s_i = ltss[i]->LabelFrom(l);
  int t_i = ltss[i]->LabelTo(l);

  if (s_i == -1) {
    int h_max = 0;

    for (s_i=0; s_i<ltss[i]->n_states(); ++s_i) {
      if (s_i == t_i) continue;
      h_max = std::max(h_max, RecursiveTauPathCost(i, s_i, t_i, ltss));
    }

    if (h_max != kInfinity) {
      ltss[i]->SetTauTransition(s_i, t_i, 1 + h_max);

      return true;
    }
  } else {
    int h = RecursiveTauPathCost(i, s_i, t_i, ltss);

    if (h != kInfinity) {
      ltss[i]->SetTauTransition(s_i, t_i, 1 + h);

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
