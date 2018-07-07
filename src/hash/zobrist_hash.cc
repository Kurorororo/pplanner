#include "hash/zobrist_hash.h"

#include <random>
#include <iostream>

namespace pplanner {

using std::vector;

ZobristHash::ZobristHash(std::shared_ptr<const SASPlus> problem, uint32_t seed)
  : n_(static_cast<int>(problem->n_variables())),
    problem_(problem) {
  std::mt19937 mt(seed);
  array_.resize(problem_->n_facts());

  for (auto &v : array_)
    v = mt();
}

uint32_t ZobristHash::operator()(const vector<int> &state) const {
  uint32_t seed = 0;

  for (int i=0, n=n_; i<n; ++i)
    seed ^= array_[problem_->Fact(i, state[i])];

  return seed;
}

uint32_t ZobristHash::HashByDifference(int action, uint32_t seed,
                                       const vector<int> &parent,
                                       const vector<int> &state) {
  auto itr = problem_->EffectVarsBegin(action);
  auto end = problem_->EffectVarsEnd(action);

  for (; itr != end; ++itr) {
    int var = *itr;
    seed ^= array_[problem_->Fact(var, parent[var])];
    seed ^= array_[problem_->Fact(var, state[var])];
  }

  return seed;
}

} // namespace pplanner
