#include "heuristics/width.h"

namespace pplanner {

using std::unordered_set;
using std::vector;

int Width::Evaluate(const vector<int> &state, int node) {
  StateToFactVector(problem_, state, tmp_facts_);
  bool width_1 = false;
  bool width_2 = false;

  for (auto f : tmp_facts_) {
    if (is_new_1_[f]) {
      is_new_1_[f] = false;
      width_1 = true;
    }

    if (is_ge_1_) continue;

    for (auto g : tmp_facts_) {
      if (f >= g || !is_new_2_[f][g]) continue;

      is_new_2_[f][g] = false;
      width_2 = true;
    }
  }

  if (width_1) return 1;

  if (is_ge_1_ || width_2) return 2;

  return 3;
}

int Width::Evaluate(const vector<int> &state, int node,
                    const vector<int> &applicable,
                    unordered_set<int> &preferred) {
  int w = Evaluate(state, node);

  preferred.clear();

  for (auto o : applicable) {
    problem_->ApplyEffect(o, state, tmp_state_);
    StateToFactVector(problem_, tmp_state_, tmp_facts_);

    for (auto f : tmp_facts_) {
      if (is_new_1_[f]) {
        preferred.insert(o);
        break;
      }

      if (is_ge_1_) continue;

      bool is_preferred = false;

      for (auto g : tmp_facts_) {
        if (f >= g || !is_new_2_[f][g]) continue;

        preferred.insert(o);
        is_preferred = true;
        break;
      }

      if (is_preferred) break;
    }
  }

  return w;
}

}  // namespace pplanner
