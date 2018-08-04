#include "dtg.h"

#include <algorithm>
#include <iostream>
#include <utility>

namespace pplanner {

using std::pair;
using std::shared_ptr;
using std::vector;

void DTG::InitTransitionLists(const vector<vector<int> > &adjacent_matrix) {
  adjacent_lists_.resize(adjacent_matrix.size());

  for (int i=0, n=adjacent_matrix.size(); i<n; ++i)
    for (int j=0; j<n; ++j)
      if (adjacent_matrix[i][j] > 0)
        adjacent_lists_[i].push_back(j);
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

void DTG::Dump() const {
  int i = 0;

  for (auto list : adjacent_lists_) {
    std::cout << i << " ->";

    for (auto value : list)
      std::cout << " " << value << "(" << adjacent_matrix_[i][value] << ")";

    std::cout << std::endl;
    ++i;
  }
}

vector<DTG> InitializeDTGs(shared_ptr<const SASPlus> problem) {
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
  }

  vector<DTG> dtgs;

  for (auto &matrix : adjacent_matrixes)
    dtgs.push_back(DTG(matrix));

  return dtgs;
}

} // namespace pplanner
