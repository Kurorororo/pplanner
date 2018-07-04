#include "hash/zobrist_hash.h"

#include <random>

namespace pplanner {

ZobristHash::ZobristHash(std::shared_ptr<const SASPlus> problem, uint32_t seed)
  : n_(static_cast<int>(problem->n_variables())),
    problem_(problem) {
  std::mt19937_64 mt(seed);
  array_.resize(problem_->n_facts());

  for (auto &v : array_)
    v = mt();
}

uint64_t ZobristHash::operator()(const std::vector<int> &state) const {
  uint64_t seed = 0;

  for (int i=0, n=n_; i<n; ++i)
    seed ^= array_[problem_->Fact(i, state[i])];

  return seed;
}

} // namespace pplanner
