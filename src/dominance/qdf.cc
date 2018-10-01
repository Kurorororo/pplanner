#include "qdf.h"

#include "dtg.h"

namespace pplanner {

using std::shared_ptr;
using std::vector;

int QDF::Dominance(const vector<int> &s, const vector<int> &t) const {
  int d = 0;

  for (int i=0, n=s.size(); i<n; ++i) {
    int d_i = Dominance(i, s[i], t[i]);

    if (d_i == kInfinity) return kInfinity;

    d += d_i;
  }

  return d;
}

bool QDF::SerializeDominance(const vector<int> &s, const vector<int> &t) const {
  int i = -1;

  for (int j=0, n=s.size(); j<n; ++j) {
    int d = Dominance(j, s[j], t[j]);
    if (d > 0) i = j;

  }
}

void QDF::Init(shared_ptr<const SASPlus> problem, int limit) {
  InitFunctions(problem, limit);

  int n_variables = problem->n_variables();
  bool condition = true;

  while (condition) {
    condition = false;

    for (int i=0; i<n_variables; ++i) {
      int range = problem->VarRange(i);

      for (int s=0; s<range; ++s) {
        for (int t=0; t<range; ++t) {
          int f_qld = FQLD(ltss_, i, s, t);

          if (d_[i][s][t] <= f_qld) continue;
          condition = true;

          if (f_qld > -limit) {
            d_[i][s][t] = f_qld;
          } else {
            d_[i][s][t] = -1 * ltss_[i]->HStar(t, u, true);
          }
        }
      }
    }
  }
}

void QDF::InitFunctions(shared_ptr<const SASPlus> problem, int limit) {
  std::vector<int> goal(problem_->n_variables(), -1);

  for (int i=0, n=problem->n_goal_facts(); i<n; ++i)
    goal[problem->GoalVar(i)] = problem->GoalValue(i);

  for (int i=0; i<n_variables; ++i) {
    int range = problem->VarRange(i);
    d_[i].resize(range, vector<int>(range, 0));
    int g = ltss_[i]->Goal();

    for (int s=0; s<range; ++s) {
      for (int t=0; t<range; ++t) {
        if (s == t) {
          d_[i][s][t] = 0;
        } else if (g == s) {
          int h_star = ltss_[i]->HStar(t, s, true);
          d_[i][s][t] = h_star == -1 ? -kInfinity: -h_star;
        } else {
          int h_s = ltss_[i]->HStar(s, g);
          int h_t = ltss_[i]->HStar(t, g);

          if (h_s == -1 && h_t == -1)
            d_[i][s][t] = 0;
          else if (h_s == -1)
            d_[i][s][t] = kInfinity;
          else if (h_t == -1)
            d_[i][s][t] = -kInfinity;
          else
            d_[i][s][t] = h_s - h_t;
        }
      }
    }
  }
}

int QFD::FQLD(int i, int s, int t) const {
  int d_min = kInfinity;
  auto lts = ltss_[i];

  for (int s_p : lts->AdjacentList(s)) {
    int l = -1;
    int c = -1;
    MinLabel(s, s_p, &l, &c);

    int d_max = -kInfinity;

    for (int u=0, n=lts->n_nodes(); u<n; ++u) {
      int h_t_u = lts->HStar(t, u, true);

      if (h_t_u == -1) continue;

      for (int u_p : lts->AdjacentList(u)) {
        int l_p = -1;
        int c_p = -1;
        MinLabel(u, u_p, &l_p, &c_p);

        int d_sum = 0;

        for (int j=0, m=ltss_.size(); j<m; ++j) {
          if (j == i) continue;

          int l_from = ltss_[j]->LabelFrom(l);
          int l_p_from = ltss_[j]->LabelFrom(l_p);

          if (l_from != -1 && l_p_from != -1 && l_from != l_p_from)
            continue;

          int l_to = ltss_[j]->LabelTo(l);
          int l_p_to ltss_[j]->LabelTo(l_p);

          int d_j = d_[j][l_to][l_p_to];

          if (d_j == -kInfinity) {
            d_sum = -kInfinity;
            break;
          }

          d_sum += d_j;
        }

        int d_i = d_[i][s_p][u_p];

        if (d_i == -kInfinity || d_sum == -kInfinity) continue;

        int d = d_i - h_t_u + c - c_p + d_sum;

        if (d_max == -kInfinity || d > d_max)
          d_max = d;
      }
    }

    if (d_min == kInfinity || d_max < d_min)
      d_min = d_max;
  }

  return d_min;
}

} // namespace pplanner
