#include "qdf.h"

#include <iostream>

namespace pplanner {

using std::shared_ptr;
using std::vector;

int QDF::QuantitativeDominance(const vector<int> &s, const vector<int> &t)
  const {
  auto &kInfinity = AtomicLTS::kInfinity;

  int d = 0;

  for (int i=0, n=s.size(); i<n; ++i) {
    int d_i = QuantitativeDominance(i, s[i], t[i]);

    if (d_i == -kInfinity) return -kInfinity;

    if (d_i == kInfinity || d == kInfinity)
      d = kInfinity;
    else
      d += d_i;
  }

  return d;
}

int QDF::QuantitativeLabelDominance(int j, int l, int l_p) const {
  auto &kInfinity = AtomicLTS::kInfinity;

  int s = ltss_[j]->LabelFrom(l);
  int t = ltss_[j]->LabelFrom(l_p);

  // t must be s or -1 (any state).
  if (t != -1 && t != s) return -kInfinity;

  int s_p = ltss_[j]->LabelTo(l);
  int s_pp = ltss_[j]->LabelTo(l_p);

  if (s_p == s_pp) return 0;

  // if s is -1 (any state), t must be -1 (any state).

  if (s == -1) {
    int d_min = kInfinity;
    int n = ltss_[j]->n_states();

    for (s=0; s<n; ++s) {
      int t_p = s_p == -1 ? s : s_p;
      int t_pp = s_pp == -1 ? s : s_pp;
      d_min = std::min(d_min, QuantitativeDominance(j, t_p, t_pp));

      if (d_min == -kInfinity) return -kInfinity;
    }

    return d_min;
  }

  // Otherwise, s is not -1.
  if (s_p == -1) s_p = s;
  // t must be -1 or s.
  if (s_pp == -1) s_pp = t == -1 ? s : t;

  return QuantitativeDominance(j, s_p, s_pp);
}

int QDF::FQLD(int i, int s, int t) const {
  auto &kInfinity = AtomicLTS::kInfinity;

  int f_qld_min = kInfinity;

  for (int s_p : ltss_[i]->To(s)) {
    for (int l : ltss_[i]->Labels(s, s_p)) {
      f_qld_min = std::min(f_qld_min, FQLDInner(i, s, s_p, l, t));
      if (f_qld_min == -kInfinity) return -kInfinity;
    }
  }

  return f_qld_min;
}

int QDF::FQLDInner(int i, int s, int s_p, int l, int t) const {
  auto &kInfinity = AtomicLTS::kInfinity;

  int c = ltss_[i]->LabelCost(l);
  int range = ltss_[i]->n_states();
  int f_qld_max = -kInfinity;

  for (int u=0; u<range; ++u) {
    int h_t_u = u == t ? 0 : ltss_[i]->ShortestPathCost(t, u, true);
    if (h_t_u == kInfinity) continue;

    for (int u_p=0; u_p<range; ++u_p) {
      int d_i = QuantitativeDominance(i, s_p, u_p);
      if (d_i == -kInfinity) continue;

      for (int l_p : ltss_[i]->Labels(u, u_p)) {
        int d_sum = 0;

        for (int j=0, m=ltss_.size(); j<m; ++j) {
          if (j == i) continue;

          int d = QuantitativeLabelDominance(j, l, l_p);

          if (d == -kInfinity) {
            d_sum = -kInfinity;
            break;
          }

          if (d_sum == kInfinity || d == kInfinity) {
            d_sum = kInfinity;
            continue;
          }

          d_sum += d;
        }

        if (d_sum == -kInfinity) continue;

        if (d_i == kInfinity || d_sum == kInfinity) return kInfinity;

        int c_p = ltss_[i]->LabelCost(l_p);
        int f_qld = d_i - h_t_u + c - c_p + d_sum;
        f_qld_max = std::max(f_qld_max, f_qld);
      }
    }
  }

  return f_qld_max;
}

void QDF::InitFunctions(int limit) {
  auto &kInfinity = AtomicLTS::kInfinity;

  for (int i=0, n=ltss_.size(); i<n; ++i) {
    int range = ltss_[i]->n_states();
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

void QDF::ComputeFunctions(int limit) {
  InitFunctions(limit);

  int n_variables = ltss_.size();
  bool condition = true;

  while (condition) {
    condition = false;

    for (int i=0; i<n_variables; ++i) {
      int range = ltss_[i]->n_states();

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

bool QDF::Dominance(const vector<int> &s, const vector<int> &t)
  const {
  for (int i=0, n=s.size(); i<n; ++i)
    if (!Dominance(i, s[i], t[i])) return false;

  return true;
}

bool QDF::LabelDominance(int j, int l, int l_p) const {
  if (l == l_p) return true;

  int s = ltss_[j]->LabelFrom(l);
  int t = ltss_[j]->LabelFrom(l_p);

  if (t != -1 && t != s) return false;

  int s_p = ltss_[j]->LabelTo(l);
  int s_pp = ltss_[j]->LabelTo(l_p);

  // if s' is s'', l <= l'.
  if (s_p == s_pp) return true;

  // if s is -1 (any state), t must be -1 (any state).

  if (s == -1) {
    int n = ltss_[j]->n_states();

    for (s=0; s<n; ++s) {
      int t_p = s_p == -1 ? s : s_p;
      int t_pp = s_pp == -1 ? s : s_pp;

      if (!Dominance(j, t_p, t_pp)) return false;
    }

    return true;
  }

  // Otherwise, s is not -1.
  if (s_p == -1) s_p = s;
  // t must be -1 or s.
  if (s_pp == -1) s_pp = t == -1 ? s : t;

  return Dominance(j, s_p, s_pp);
}

void QDF::InitRelations() {
  int n_variables = ltss_.size();

  for (int i=0; i<n_variables; ++i) {
    int range = ltss_[i]->n_states();
    r_[i].resize(range, vector<bool>(range, false));
    int g = ltss_[i]->goal();

    for (int s=0; s<range; ++s)
      for (int t=0; t<range; ++t)
        if (s != g || t == g)
          r_[i][s][t] = true;
  }
}

bool QDF::Ok(int i, int s, int t) const {
  if (s == t) return true;

  if (s == ltss_[i]->goal() && t != ltss_[i]->goal())
    return false;

  for (int s_p : ltss_[i]->To(s)) {
    int c_min = ltss_[i]->TransitionCost(s, s_p);

    for (int t_p : ltss_[i]->To(t)) {
      int c_p_min = ltss_[i]->TransitionCost(t, t_p);

      if (c_p_min > c_min || !Dominance(i, s_p, t_p)) return false;

      for (auto l : ltss_[i]->Labels(s, s_p)) {
        int c = ltss_[i]->LabelCost(l);
        bool dominated = false;

        for (auto l_p : ltss_[i]->Labels(t, t_p)) {
          int c_p = ltss_[i]->LabelCost(l_p);

          if (c_p > c) continue;

          bool label_dominance = true;

          for (int j=0, n=ltss_.size(); j<n; ++j) {
            if (i == j || LabelDominance(j, l, l_p)) continue;
            label_dominance = false;
            break;
          }

          if (label_dominance) {
            dominated = true;
            break;
          }
        }

        if (!dominated) return false;
      }
    }
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
  auto &kInfinity = AtomicLTS::kInfinity;

  std::cout << "Qualitative" << std::endl;

  for (int i=0, n=ltss_.size(); i<n; ++i) {
    std::cout << "var" << i << std::endl;
    int range = ltss_[i]->n_states();

    for (int s=0; s<range; ++s) {
      std::cout << s << " <= ";

      for (int t=0; t<range; ++t) {
        if (Dominance(i, s, t))
          std::cout << t << ", ";
      }

      std::cout << std::endl;
    }
  }

  std::cout << "Quantitative" << std::endl;

  for (int i=0, n=ltss_.size(); i<n; ++i) {
    std::cout << "var" << i  << std::endl;
    int range = ltss_[i]->n_states();

    for (int s=0; s<range; ++s) {
      for (int t=0; t<range; ++t) {
        std::cout << "D_" << i << "(" << s << ", "<< t << ")=";
        int d = QuantitativeDominance(i, s, t);

        if (d == kInfinity)
          std::cout << "inf., ";
        else if (d == -kInfinity)
          std::cout << "-inf., ";
        else
          std::cout << d << ", ";
      }

      std::cout << std::endl;
    }
  }
}

} // namespace pplanner
