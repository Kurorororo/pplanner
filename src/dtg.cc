#include "dtg.h"

#include <algorithm>
#include <iostream>
#include <utility>

namespace pplanner {

using std::pair;
using std::shared_ptr;
using std::vector;

void DTG::RemoveNode(int value) {
  adjacent_lists_[value].clear();

  for (auto &list : adjacent_lists_)
    list.erase(std::remove(list.begin(), list.end(), value), list.end());
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
      std::cout << " " << value;

    std::cout << std::endl;
    ++i;
  }
}

vector<DTG> InitializeDTGs(shared_ptr<const SASPlus> problem) {
  int variables_size = problem->n_variables();
  vector<vector<vector<int> > > lists_lists(variables_size);

  for (size_t i=0; i<variables_size; ++i)
    lists_lists[i].resize(problem->VarRange(i));

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
          auto &list = lists_lists[j][k];

          if (std::find(list.begin(), list.end(), effect_value) == list.end())
            list.push_back(effect_value);
        }
      } else {
        auto &list = lists_lists[j][precondition_value];

        if (std::find(list.begin(), list.end(), effect_value) == list.end())
          list.push_back(effect_value);
      }
    }
  }

  vector<DTG> dtgs;

  for (auto &list : lists_lists)
    dtgs.push_back(DTG(list));

  return dtgs;
}

} // namespace pplanner
