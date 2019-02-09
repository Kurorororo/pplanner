#include "postprocess/action_elimination.h"

#include <utility>

namespace pplanner {

using std::pair;
using std::vector;

vector<int> ActionElimination(const SASPlus &problem, vector<int> plan) {
  auto state = problem.initial();

  int i = 0;
  int n = plan.size();
  vector<bool> marked(plan.size(), false);

  while (i < n) {
    marked[i] = true;
    auto successor = state;
    auto tmp = state;
    vector<pair<int, int> > precondition;

    for (int j=i+1; j<n; ++j) {
      problem.CopyPrecondition(plan[j], precondition);

      for (auto p : precondition) {
        if (successor[p.first] != p.second) {
          marked[j] = true;
          break;
        }
      }

      if (!marked[j]) problem.ApplyEffect(plan[j], successor, tmp);
      successor = tmp;
    }

    if (problem.IsGoal(successor)) {
      vector<int> new_plan;

      for (int j=0; j<n; ++j)
        if (!marked[j]) new_plan.push_back(plan[j]);

      plan.swap(new_plan);
      marked.resize(plan.size());
      n = plan.size();
      continue;
    } else {
      problem.ApplyEffect(plan[i], state, tmp);
      state = tmp;
      ++i;
    }

    std::fill(marked.begin(), marked.end(), false);
  }

  return plan;
}


}; // namespace ppalnner
