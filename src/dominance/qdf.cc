#include "qdf.h"

#include "dtg.h"

namespace pplanner {

using std::shared_ptr;
using std::vector;

int QDF::Dominance(const vector<int> &s, const vector<int> &t) const {
  int d = 0;

  for (int i=0, n=s.size(); i<n; ++i)
    d += Dominance(i, s[i], t[i]);

  return d;
}

void QDF::Init(shared_ptr<const SASPlus> problem, int limit) {
  int n_variables = problem->n_variables();
  std::vector<int> goal(n_variables, -1);

  for (int i=0, n=problem->n_goal_facts(); i<n; ++i)
    goal[problem->GoalVar(i)] = problem->GoalValue(i);

  auto ltss = InitializeLTSs(problem);

  for (int i=0; i<n_variables; ++i) {
    int range = problem->VarRange(i);
    d_[i].resize(range, vector<int>(range));

    for (int s=0; s<range; ++s) {
      for (int t=0; t<range; ++t) {
        if (ltss[i]->Goal() == s)
          d_[i][s][t] = -1 * ltss[i]->HStar(t, true);
        else
          d_[i][s][t] = ltss[i]->HStar(s) - ltss[i]->HStar(t);
      }
    }
  }

  bool condition = false;

  while (condition) {
    condition = false;

    for (int i=0; i<n_variables; ++i) {
      int range = problem->VarRange(i);

      for (int s=0; s<range; ++s) {
        for (int t=0; t<range; ++t) {
          int f_qld = FQLD(ltss, i, s, t);

          if (d_[i][s][t] <= f_qld) continue;
          condition = true;

          if (f_qld > -1 * limit)
            d_[i][s][t] = f_qld;
          else
            d_[i][s][t] = -1 * ltss[i]->HStar(t, u, true);
        }
      }
    }
  }
}

int QFD::FQLD(const vector<shared_ptr<LTS> > &ltss, int i, int s, int t) const {
  int d_min = -1;
  auto lts = ltss[i];

  for (int s_p : lts->AdjacentList(s)) {
    int l = -1;
    int c = -1;
    MinLabel(s, s_p, &l, &c);

    int d_max = -1;

    for (int u=0, n=lts->n_nodes(); u<n; ++u) {
      int h_t_u = lts->HStar(t, u, true);

      for (int u_p : lts->AdjacentList(u)) {
        int l_p = -1;
        int c_p = -1;
        MinLabel(u, u_p, &l_p, &c_p);

        int d_sum = 0;

        for (int j=0, m=ltss.size(); j<m; ++j) {
          if (j == i) continue;

          int l_from = ltss[j]->LabelFrom(l);
          int l_p_from = ltss[j]->LabelFrom(l_p);

          if (l_from != -1 && l_p_from != -1 && l_from != l_p_from)
            continue;

          int l_to = ltss[j]->LabelTo(l);
          int l_p_to ltss[j]->LabelTo(l_p);

          d_sum += d_[j][l_to][l_p_to];
        }

        int d = d_[i][s_p][u_p] - h_t_u + c - c_p + d_sum;

        if (d_max == -1 || d > d_max)
          d_max = d;
      }
    }

    if (d_min == -1 || d_max < d_min)
      d_min = d_max;
  }

  return d_min;
}

} // namespace pplanner
