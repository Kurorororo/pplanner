#include "lds.h"

#include <iostream>

namespace pplanner {

using std::vector;

bool LDS::Dominance(const vector<int> &s, const vector<int> &t)
  const {
  for (int i=0, n=s.size(); i<n; ++i)
    if (!Dominance(i, s[i], t[i])) return false;

  return true;
}

bool LDS::LabelDominance(int j, int l, int l_p) const {
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

void LDS::InitRelations() {
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


bool LDS::Ok(int i, int s, int t) const {
  if (s == t) return true;

  if (s == ltss_[i]->goal() && t != ltss_[i]->goal())
    return false;

  for (int s_p : ltss_[i]->To(s)) {
    bool dominated = true;

    for (int t_p : ltss_[i]->To(t)) {
      if (!TransisionDominance(i, s, s_p, t, t_p)) {
        dominated = false;
        break;
      }
    }

    if (!dominated && !TransisionDominance(i, s, s_p, t, t))
      return false;
  }

  return true;
}

bool LDS::TransisionDominance(int i, int s, int s_p, int t, int t_p) const {
  int c_min = ltss_[i]->TransitionCost(s, s_p);
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

  return true;
}

void LDS::ComputeRelations() {
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

void LDS::Dump() const {
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
}

size_t LDS::n_bytes() const {
  size_t size = 0;

  for (auto v : ltss_)
    size += v->n_bytes();

  for (auto &v : r_)
    size += v.size() * v.size() * sizeof(bool);

  return size;
}

} // namespace pplanner
