#include "hash/gra_zobrist_hash.h"

#include <random>

#include "dtg.h"

namespace pplanner {

using std::vector;

GRAZobristHash::GRAZobristHash(std::shared_ptr<const SASPlus> problem,
                               uint32_t seed, bool greedy)
  : problem_(problem), array_(problem->n_variables()) {
  auto dtgs = InitializeDTGs(problem);
  std::vector<int> cut;

  for (auto &dtg : dtgs) {
    if (greedy || dtg.n_nodes() > 20)
      dtg.GreedyCut(cut);
    else
      dtg.SparsestCut(cut);

    cuts_.push_back(cut);
  }

  std::mt19937 mt(seed);

  for (auto &a : array_)
    for (auto &v : a)
      v = mt();
}

uint32_t GRAZobristHash::operator()(const vector<int> &state) const {
  uint32_t seed = 0;

  for (int i=0, n=state.size(); i<n; ++i)
    seed ^= array_[i][cuts_[i][state[i]]];

  return seed;
}

uint32_t GRAZobristHash::HashByDifference(int action, uint32_t seed,
                                          const vector<int> &parent,
                                          const vector<int> &state) {
  auto itr = problem_->EffectVarsBegin(action);
  auto end = problem_->EffectVarsEnd(action);

  for (; itr != end; ++itr) {
    int var = *itr;
    seed = seed ^ array_[var][cuts_[var][parent[var]]]
                ^ array_[var][cuts_[var][state[var]]];
  }

  return seed;
}

} // namespace pplanner
