#include "dtg.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "utils/arg_sort.h"

namespace pplanner {

using std::pair;
using std::shared_ptr;
using std::vector;

void DTG::Init(const vector<vector<int> > &adjacent_matrix) {
  adjacent_lists_.resize(adjacent_matrix.size());
  in_degrees_.resize(adjacent_matrix.size(), 0);
  out_degrees_.resize(adjacent_matrix.size(), 0);

  for (int i=0, n=adjacent_matrix.size(); i<n; ++i) {
    for (int j=0; j<n; ++j) {
      int weight = adjacent_matrix[i][j];

      if (weight > 0) {
        adjacent_lists_[i].push_back(j);
        out_degrees_[i] += weight;
        in_degrees_[j] += weight;
      }
    }
  }

  vector<int> degrees(adjacent_matrix.size(), 0);

  for (int i=0, n=in_degrees_.size(); i<n; ++i)
    degrees[i] = out_degrees_[i] + in_degrees_[i];

  ArgSort(degrees, nodes_by_degree_, true);
}

void DTG::RemoveNode(int value) {
  adjacent_lists_[value].clear();

  for (auto &list : adjacent_lists_)
    list.erase(std::remove(list.begin(), list.end(), value), list.end());

  adjacent_matrix_[value].clear();

  for (auto &row : adjacent_matrix_)
    row[value] = 0;
}

bool DTG::RecursiveIsConnected(int i, int goal) {
  if (i == goal) return true;

  for (auto j : adjacent_lists_[i]) {
    if (closed_.find(j) != closed_.end()) continue;
    if (deleted_.find(j) != deleted_.end()) continue;
    closed_.insert(j);
    if (RecursiveIsConnected(j, goal)) return true;
  }

  return false;
}

bool DTG::IsConnected(int start, int goal, int ignore) {
  assert(goal != ignore);
  if (start == ignore || deleted_.find(start) != deleted_.end()) return false;
  closed_.clear();
  closed_.insert(ignore);

  return RecursiveIsConnected(start, goal);
}

double DTG::GreedyCut(vector<int> &cut) const {
  int n_nodes = adjacent_lists_.size();

  cut.resize(n_nodes);
  std::fill(cut.begin(), cut.end(), 0);
  vector<int> count(n_nodes, 0);

  for (int i=0; i<n_nodes; ++i) {
    for (int j : adjacent_lists_[i]) {
      count[i] += adjacent_matrix_[i][j];
      count[j] += adjacent_matrix_[i][j];
    }
  }

  int least = -1;
  int arg_least = -1;

  for (int i=0; i<n_nodes; ++i) {
    if (count[i] < least || least == -1) {
      least = count[i];
      arg_least = i;
    }
  }

  cut[arg_least] = 1;
  int limit = n_nodes / 2;

  for (int i=1; i<limit; ++i) {
    std::vector<int> count(n_nodes, 0);

    for (int j=0; j<n_nodes; ++j) {
      if (cut[j] == 1) continue;

      for (int k : adjacent_lists_[j]) {
        if (cut[k] == 0) continue;

        count[j] += adjacent_matrix_[j][k];
        count[k] += adjacent_matrix_[j][k];
      }
    }

    int most = -1;
    int arg_most = -1;

    for (int j=0; j<n_nodes; ++j) {
      if (cut[j] == 1) continue;

      if (count[j] > most) {
        most = count[j];
        arg_most = j;
      }
    }

    cut[arg_most] = 1;
  }

  return CalculateSparsity(cut);
}

double DTG::SparsestCut(vector<int> &cut, int max_expansion) const {
  vector<int> greedy_cut(n_nodes());
  double greedy_answer = GreedyCut(greedy_cut);

  cut.resize(n_nodes());
  std::fill(cut.begin(), cut.end(), -1);
  max_expansion = std::max(max_expansion, n_nodes());
  int count = max_expansion;
  double answer = RecursiveSparsestCut(0, 0.0, cut, &count);

  if (std::find(cut.begin(), cut.end(), -1) != cut.end()
      || answer < greedy_answer) {
    cut = greedy_cut;
    answer = greedy_answer;
  }

  return answer;
}

double DTG::RecursiveSparsestCut(int index, double answer, vector<int> &cut,
                                 int *count) const {
  if (--(*count) < 0) return answer;

  if (index == static_cast<int>(n_nodes()))
    return CalculateSparsity(cut);

  double ub = CalculateUpperBound(cut);

  if (ub <= answer) return -1.0;

  int value = nodes_by_degree_[index];

  auto zero_cut = cut;
  zero_cut[value] = 0;
  double zero_answer = RecursiveSparsestCut(index + 1, answer, zero_cut, count);

  if (zero_answer > answer) answer = zero_answer;

  auto one_cut = cut;
  one_cut[value] = 1;
  double one_answer = RecursiveSparsestCut(index + 1, answer, one_cut, count);

  if (one_answer > zero_answer) {
    cut = one_cut;

    return one_answer;
  }

  cut = zero_cut;

  return zero_answer;
}

double DTG::CalculateSparsity(const vector<int> &cut) const {
  int n_zero_nodes = 0.0;
  int n_one_nodes = 0.0;
  int cut_edge_weight = 0.0;
  double total_weight = 0.0;

  for (int i=0, n=cut.size(); i<n; ++i) {
    if (cut[i] == 0) ++n_zero_nodes;
    if (cut[i] == 1) ++n_one_nodes;

    for (auto j : adjacent_lists_[i]) {
      total_weight += static_cast<double>(adjacent_matrix_[i][j]);

      if (cut[i] != cut[j])
        cut_edge_weight += adjacent_matrix_[i][j];
    }
  }

  double n_total_nodes = static_cast<double>(cut.size());
  double s_0 = static_cast<double>(n_zero_nodes) / n_total_nodes;
  double s_1 = static_cast<double>(n_one_nodes) / n_total_nodes;

  if (cut_edge_weight == 0) return 0.0;

  double e = static_cast<double>(cut_edge_weight) / total_weight;

  return s_0 * s_1 / e;
}

double DTG::CalculateUpperBound(const vector<int> &cut) const {
  int n_zero_nodes = 0.0;
  int n_one_nodes = 0.0;
  int n_yet_nodes = 0.0;
  double cut_edge_weight = 0.0;
  double total_weight = 0.0;

  for (int i=0, n=cut.size(); i<n; ++i) {
    if (cut[i] == 0) ++n_zero_nodes;
    if (cut[i] == 1) ++n_one_nodes;
    if (cut[i] == -1) ++n_yet_nodes;

    for (auto j : adjacent_lists_[i]) {
      total_weight += static_cast<double>(adjacent_matrix_[i][j]);

      if (cut[i] != cut[j] && cut[i] != -1 && cut[j] != -1)
        cut_edge_weight += static_cast<double>(adjacent_matrix_[i][j]);
    }
  }

  for (int i=n_yet_nodes; i>0; --i) {
    if (n_zero_nodes < n_one_nodes)
      ++n_zero_nodes;
    else
      ++n_one_nodes;
  }

  double n_total_nodes = static_cast<double>(cut.size());
  double s_0 = static_cast<double>(n_zero_nodes) / n_total_nodes;
  double s_1 = static_cast<double>(n_one_nodes) / n_total_nodes;
  cut_edge_weight = std::max(cut_edge_weight, 0.0000001);
  double e = cut_edge_weight / total_weight;

  return s_0 * s_1 / e;
}

void DTG::Dump() const {
  int i = 0;

  for (auto list : adjacent_lists_) {
    std::cout << i << " ->";

    for (auto value : list)
      std::cout << " " << value << "(" << adjacent_matrix_[i][value] << ")";

    std::cout << ", #in=" << InDegree(i) << ", #out=" << OutDegree(i);
    std::cout << std::endl;
    ++i;
  }

  std::vector<int> cut;

  double sparsity = SparsestCut(cut);
  std::cout << "sparsest cut: ";

  for (int i=0, n=cut.size(); i<n; ++i)
    std::cout << i << ":" << cut[i] << " ";

  std::cout << "sparsity:" << sparsity << std::endl;

  sparsity = GreedyCut(cut);
  std::cout << "greedy cut: ";

  for (int i=0, n=cut.size(); i<n; ++i)
    std::cout << i << ":" << cut[i] << " ";

  std::cout << "sparsity:" << sparsity << std::endl;
}

vector<shared_ptr<DTG> > InitializeDTGs(shared_ptr<const SASPlus> problem) {
  int variables_size = problem->n_variables();
  vector<vector<vector<int> > > adjacent_matrixes(variables_size);

  for (int i=0; i<variables_size; ++i) {
    int var_range = problem->VarRange(i);
    adjacent_matrixes[i].resize(var_range);

    for (int j=0; j<var_range; ++j)
      adjacent_matrixes[i][j].resize(var_range, 0);
  }

  vector<int> precondition_table(variables_size);
  vector<int> effect_table(variables_size);
  vector<pair<int, int> > tmp_precondition;
  vector<pair<int, int> > tmp_effect;
  vector<vector<pair<int, int> > > tmp_effect_conditions;
  vector<pair<int, int> > tmp_condtional_effects;

  for (int i=0, n=problem->n_actions(); i<n; ++i) {
    std::fill(precondition_table.begin(), precondition_table.end(), -1);
    std::fill(effect_table.begin(), effect_table.end(), -1);

    problem->CopyPrecondition(i, tmp_precondition);

    for (auto v : tmp_precondition)
      precondition_table[v.first] = v.second;

    problem->CopyEffect(i, tmp_effect);

    for (auto v : tmp_effect)
      effect_table[v.first] = v.second;

    for (int j=0; j<variables_size; ++j) {
      int precondition_value = precondition_table[j];
      int effect_value = effect_table[j];
      if (effect_value == -1 || precondition_value == effect_value) continue;

      if (precondition_value == -1) {
        for (int k=0; k<problem->VarRange(j); ++k) {
          if (k == effect_value) continue;
          ++adjacent_matrixes[j][k][effect_value];
        }
      } else {
        ++adjacent_matrixes[j][precondition_value][effect_value];
      }
    }

    if (problem->HasConditionalEffects(i)) {
      problem->CopyEffectConditions(i, tmp_effect_conditions);
      problem->CopyConditionalEffects(i, tmp_condtional_effects);

      for (int j=0, m=tmp_effect_conditions.size(); j<m; ++j) {
        int effect_var = tmp_condtional_effects[j].first;
        int effect_value = tmp_condtional_effects[j].second;
        int precondition_value = effect_table[effect_var];

        if (precondition_value != -1) {
          ++adjacent_matrixes[effect_var][precondition_value][effect_value];
          continue;
        }

        bool found = false;

        for (int k=0, l=tmp_effect_conditions.size(); k<l; ++k) {
          int precondition_var = tmp_effect_conditions[j][k].first;

          if (precondition_var == effect_var) {
            int precondition_value = tmp_effect_conditions[j][k].second;
            ++adjacent_matrixes[effect_var][precondition_value][effect_value];
            found = true;
            break;
          }
        }

        if (!found) {
          for (int k=0; k<problem->VarRange(effect_var); ++k) {
            if (k == effect_value) continue;
            ++adjacent_matrixes[effect_var][k][effect_value];
          }
        }
      }
    }
  }

  vector<shared_ptr<DTG> > dtgs;

  for (auto &matrix : adjacent_matrixes)
    dtgs.push_back(std::make_shared<DTG>(matrix));

  return dtgs;
}

} // namespace pplanner
