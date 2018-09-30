#include "atomic_lts.h"

namespace pplanner {

shared_ptr<const SASPlus> AtomicLTS::problem_ = nullptr;

void AtomicLTS::MinLabel(int s, int t, int *label, int *cost, bool only_tau)
  const {
  *label = -1;
  *cost = -1;

  for (auto l : labels_[s][t]) {
    if (only_tau && !tau_[l]) continue;

    int c = problem_->ActionCost(l);

    if (*cost == -1 || c < *cost) {
      *label = l;
      *cost = c;
    }
  }
}

int AtomicLTS::HStar(int start, bool only_tau) const {
  if (h_star_cache_[start] != -1) return h_star_cache_[start];

  open_ = PQueue();
  std::fill(closed_.begin(). closed_.end(), -1);

  open_.push(std::make_pair(0, start));

  while (!open__.empty()) {
    auto top = open_.top();
    open_.pop();

    int g = top.first;
    int s = top.second;

    if (closed_[v] < g) continue;

    if (s == Goal()) {
      h_star_cache_[start] = g;

      return g;
    }

    for (auto t : AdjacentList(s)) {
      int l = -1;
      int c = -1;
      MinLabel(s, t, &l, &c, only_tau);

      if (c == -1) continue;

      int cost = g + c;

      if (closed_[t] == -1 || cost < closed_[t]) {
        closed_[t] = cost;
        open_.push(std::make_pair(cost, t));
      }
    }
  }

  h_star_cache_[start] = -1;

  return -1;
}

void Init() {
  int n = n_nodes();

  for (int s=0; s<n; ++s) {
    for (int t=0; t<n; ++t) {
      for (auto l : labels_[s][t]) {
        label_from_[l] = s;
        label_to_[l] = t;
      }
    }
  }
}

vector<shared_ptr<LTS> > InitializeLTSs(shared_ptr<const SASPlus> problem) {
  int variables_size = problem->n_variables();
  vector<vector<vector<int> > > adjacent_matrixes(variables_size);
  vector<vector<vector<vector<int> > > > labels(variables_size);
  vector<vector<bool> > taus(variables_size,
                             vector<bool>(variables_size, false));

  for (int i=0; i<variables_size; ++i) {
    int var_range = problem->VarRange(i);
    adjacent_matrixes[i].resize(var_range);
    labels[i].resize(var_range);
    tau[i].resize(var_range)

    for (int j=0; j<var_range; ++j) {
      adjacent_matrixes[i][j].resize(var_range, 0);
      labels[i][j].resize(var_range, vector<int>());
    }
  }

  vector<int> precondition_table(variables_size);
  vector<int> effect_table(variables_size);
  vector<pair<int, int> > tmp_precondition;
  vector<pair<int, int> > tmp_effect;

  for (int i=0, n=problem->n_actions(); i<n; ++i) {
    std::fill(precondition_table.begin(), precondition_table.end(), -1);
    std::fill(effect_table.begin(), effect_table.end(), -1);

    problem->CopyPrecondition(i, tmp_precondition);

    for (auto v : tmp_precondition)
      precondition_table[v.first] = v.second;

    problem->CopyEffect(i, tmp_effect);

    for (auto v : tmp_effect)
      effect_table[v.first] = v.second;

    int tau_v = -1;

    for (int j=0; j<variables_size; ++j) {
      if (precondition_table[j] == -1 && effect_table[j] == -1) continue;

      if (tau_v == -1) {
        tau_v = j;
      } else {
        tau_v = -1;
        break;
      }
    }

    if (tau_v != -1) taus[tau_v][i] = true;

    for (int j=0; j<variables_size; ++j) {
      int precondition_value = precondition_table[j];
      int effect_value = effect_table[j];
      if (effect_value == -1 || precondition_value == effect_value) continue;

      if (precondition_value == -1) {
        for (int k=0; k<problem->VarRange(j); ++k) {
          if (k == effect_value) continue;
          ++adjacent_matrixes[j][k][effect_value];
          labels[j][k][effect_value].push_back(i);
        }
      } else {
        ++adjacent_matrixes[j][precondition_value][effect_value];
        labels[j][precondition_value][effect_value].push_back(i);
      }
    }
  }

  auto initial = problem->Initial();
  std::vector<int> goal(variables_size, -1);

  for (int i=0, n=problem->n_goal_facts(); i<n; ++i)
    goal[problem->GoalVar(i)] = problem->GoalValue(i);

  vector<shared_ptr<LTS> > ltss;

  for (int i=0; i<variables_size; ++i) {
    ltss.push_back(std::make_shared<LTS>(
          matrixes[i], initial[i], goal[i], labels[i], taus[i]);
  }

  return ltss;
}

} // namespace pplanner
