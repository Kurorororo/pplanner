#include "landmark/dtg.h"

#include <algorithm>
#include <iostream>

using std::vector;

namespace rwls {

void DTG::RemoveNode(int value) {
  adjacent_lists_[value].clear();
  for (auto &list : adjacent_lists_)
    list.erase(value);
}

void DTG::RemoveNodesByRRPG(const Domain &domain, const PlanningGraph &rrpg,
                            int var, int goal_value) {
  RecoverSoftDelete();
  for (int i=0, m=domain.dom[var]; i<m; ++i) {
    if (i == goal_value) continue;
    int f = ToFact(domain.fact_offset, var, i);
    if (rrpg.fact_layer_membership[f] == -1) SoftRemoveNode(i);
  }
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

void DTG::Print() {
  int i = 0;
  for (auto list : adjacent_lists_) {
    std::cout << i << " ->";
    for (auto value : list)
      std::cout << " " << value;
    std::cout << std::endl;
    ++i;
  }
}

vector<DTG> DTG::InitializeDTGs(const Domain &domain) {
  size_t variables_size = domain.variables_size;
  vector<DTG> dtgs(variables_size);
  for (size_t i=0; i<variables_size; ++i)
    dtgs[i].adjacent_lists_.resize(domain.dom[i]);
  vector<int> tmp_preconditions(variables_size);
  vector<int> tmp_effects(variables_size);

  for (size_t i=0, n=domain.action_size; i<n; ++i) {
    std::fill(tmp_preconditions.begin(), tmp_preconditions.end(), -1);
    std::fill(tmp_effects.begin(), tmp_effects.end(), -1);
    for (auto v : domain.preconditions[i]) {
      int var, value;
      DecodeVarValue(v, &var, &value);
      tmp_preconditions[var] = value;
    }
    for (auto v : domain.effects[i]) {
      int var, value;
      DecodeVarValue(v, &var, &value);
      tmp_effects[var] = value;
    }
    for (size_t j=0; j<variables_size; ++j) {
      int effect_value = tmp_effects[j];
      int precondition_value = tmp_preconditions[j];
      if (effect_value == -1 || precondition_value == effect_value) continue;
      if (precondition_value == -1) {
        for (int k=0; k<domain.dom[j]; ++k) {
          if (k == effect_value) continue;
          dtgs[j].adjacent_lists_[k].insert(effect_value);
        }
      } else {
        dtgs[j].adjacent_lists_[precondition_value].insert(effect_value);
      }
    }
  }
  return dtgs;
}

} // namespace rwls
