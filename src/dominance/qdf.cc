#include "qdf.h"

#include <iostream>

namespace pplanner {

using std::shared_ptr;
using std::vector;

int QDF::Dominance(const vector<int> &s, const vector<int> &t) const {
  auto &kInfinity = AtomicLTS::kInfinity;

  int d = 0;

  for (int i=0, n=s.size(); i<n; ++i) {
    int d_i = Dominance(i, s[i], t[i]);

    if (d_i == -kInfinity) return -kInfinity;

    if (d_i == kInfinity || d == kInfinity)
      d = kInfinity;
    else
      d += d_i;
  }

  return d;
}

int QDF::LabelDominance(int j, int l, int l_p) const {
  auto &kInfinity = AtomicLTS::kInfinity;

  int s = ltss_[j]->LabelFrom(l_p);

  if (s != -1 && s != ltss_[j]->LabelFrom(l))
    return -kInfinity;

  int s_p = ltss_[j]->LabelTo(l);
  int s_pp = ltss_[j]->LabelTo(l_p);

  if (s_p == s_pp) return 0;

  if (s == -1) {
    int d_min = kInfinity;
    int range = ltss_[j]->n_states();

    for (s=0; s<range; ++s) {
      int t_p = s_p == -1 ? s : s_p;
      int t_pp = s_pp == -1 ? s : s_pp;
      d_min = std::min(d_min, Dominance(j, t_p, t_pp));
    }

    return d_min;
  }

  if (s_p == -1) s_p = s;
  if (s_pp == -1) s_pp = s;

  return Dominance(j, s_p, s_pp);
}

int QDF::FQLD(int i, int s, int t) const {
  auto &kInfinity = AtomicLTS::kInfinity;

  int f_qld_min = kInfinity;

  for (int l : ltss_[i]->Labels(s)) {
    int s_p = ltss_[i]->LabelTo(l);
    if (s_p == -1) s_p = s;

    int f_qld = FQLDInner(i, s, t, l, s_p);
    if (f_qld == -kInfinity) return -kInfinity;

    f_qld_min = std::min(f_qld_min, f_qld);
  }

  return f_qld_min;
}

int QDF::FQLDInner(int i, int s, int t, int l, int s_p) const {
  auto &kInfinity = AtomicLTS::kInfinity;

  int f_qld_max = -kInfinity;

  for (int u=0, n=ltss_[i]->n_states(); u<n; ++u) {
    int h_t_u = ltss_[i]->ShortestPathCost(t, u, true);
    if (h_t_u == kInfinity) continue;

    for (auto l_p : ltss_[i]->Labels(u)) {
      int u_p = ltss_[i]->LabelTo(l_p);
      if (u_p == -1) u_p = u;

      int d_i = Dominance(i, s_p, u_p);
      if (d_i == -kInfinity) continue;
      if (d_i == kInfinity) return kInfinity;

      int d_sum = 0;

      for (int j=0, m=ltss_.size(); j<m; ++j) {
        if (j == i) continue;
        int d = LabelDominance(j, l, l_p);

        if (d == -kInfinity) {
          d_sum = -kInfinity;
          break;
        }

        if (d_sum != kInfinity && d != kInfinity)
          d_sum += d;
        else
          d_sum = kInfinity;
      }

      if (d_sum == -kInfinity) continue;
      if (d_sum == kInfinity) return kInfinity;

      f_qld_max = std::max(f_qld_max, d_i - h_t_u + d_sum);
    }
  }

  return f_qld_max;
}

void QDF::InitFunctions(shared_ptr<const SASPlus> problem, int limit) {
  auto &kInfinity = AtomicLTS::kInfinity;

  int n_variables = problem->n_variables();
  std::vector<int> goal(n_variables, -1);

  for (int i=0, n=problem->n_goal_facts(); i<n; ++i)
    goal[problem->GoalVar(i)] = problem->GoalValue(i);

  for (int i=0; i<n_variables; ++i) {
    int range = problem->VarRange(i);
    d_[i].resize(range, vector<int>(range, 0));
    int g = ltss_[i]->goal();

    for (int s=0; s<range; ++s) {
      for (int t=0; t<range; ++t) {
        if (s == t) {
          d_[i][s][t] = 0;
        } else if (g == s) {
          d_[i][s][t] = -ltss_[i]->ShortestPathCost(t, s, true);
        } else {
          int h_s = ltss_[i]->ShortestPathCost(s, g);
          int h_t = ltss_[i]->ShortestPathCost(t, g);

          if (h_s == kInfinity && h_t != kInfinity)
            d_[i][s][t] = kInfinity;
          else if (h_s != kInfinity && h_t == kInfinity)
            d_[i][s][t] = -kInfinity;
          else
            d_[i][s][t] = h_s - h_t;
        }
      }
    }
  }
}

void QDF::ComputeFunctions(shared_ptr<const SASPlus> problem, int limit) {
  InitFunctions(problem, limit);

  int n_variables = problem->n_variables();
  bool condition = true;

  while (condition) {
    condition = false;

    for (int i=0; i<n_variables; ++i) {
      int range = problem->VarRange(i);

      for (int s=0; s<range; ++s) {
        for (int t=0; t<range; ++t) {
          if (s == t) continue;

          int f_qld = FQLD(i, s, t);

          if (d_[i][s][t] <= f_qld) continue;
          condition = true;

          if (f_qld > -limit)
            d_[i][s][t] = f_qld;
          else
            d_[i][s][t] = -ltss_[i]->ShortestPathCost(t, s, true);
        }
      }
    }
  }
}

bool QDF::QualitativeDominance(const vector<int> &s, const vector<int> &t)
  const {
  for (int i=0, n=s.size(); i<n; ++i)
    if (!QualitativeDominance(i, s[i], t[i])) return false;

  return true;
}

void QDF::InitRelations() {
  int n_variables = ltss_.size();

  for (int i=0; i<n_variables; ++i) {
    int range = ltss_[i]->n_states();
    r_[i].resize(range, vector<bool>(range, false));
    int g = ltss_[i]->goal();

    for (int s=0; s<range; ++s)
      for (int t=0; t<range; ++t)
        if (s == t || s != g || t == g)
          r_[i][s][t] = true;
  }
}

bool QDF::Ok(int i, int s, int t) const {
  if (s == t) return true;

  if (s == ltss_[i]->goal() && t != ltss_[i]->goal())
    return false;

  for (auto l : ltss_[i]->Labels(s)) {
    int s_p = ltss_[i]->LabelTo(l);
    if (s_p == -1) s_p = s;
    int c = ltss_[i]->LabelCost(l);
    bool all = true;

    for (auto l_p : ltss_[i]->Labels(t)) {
      int t_p = ltss_[i]->LabelTo(l_p);
      if (t_p == -1) t_p = t;
      int c_p = ltss_[i]->LabelCost(l_p);

      if (c_p <= c && r_[i][s_p][t_p]) {
        all = false;
        break;
      }
    }

    if (all) return false;
  }

  return true;
}

void QDF::ComputeRelations() {
  InitRelations();

  int n_variables = ltss_.size();
  bool condition = true;

  while (condition) {
    condition = false;

    for (int i=0; i<n_variables; ++i) {
      int range = ltss_[i]->n_states();

      for (int s=0; s<range; ++s) {
        for (int t=0; t<range; ++t) {
          if (r_[i][s][t] && !Ok(i, s, t)) {
            r_[i][s][t] = false;
            condition = true;
          }
        }
      }
    }
  }
}

void QDF::Dump() const {
  for (int i=0, n=ltss_.size(); i<n; ++i) {
    std::cout << "var" << i << std::endl;
    int range = ltss_[i]->n_states();

    for (int s=0; s<range; ++s)
      for (int t=0; t<range; ++t)
        if (r_[i][s][t])
          std::cout << s << "<=" << t << std::endl;
  }
}

} // namespace pplanner
