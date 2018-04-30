#include "hash/zobrist_hash.h"

#include <random>

namespace pplanner {

ZobristHash::ZobristHash(std::shared_ptr<const SASPlus> problem)
  : n_(static_cast<int>(problem->n_variables())),
    problem_(problem) {
  // std::random_device rng;
  // std::mt19937_64 mt(rng());
  std::mt19937_64 mt(2886379259);
  array_.resize(problem_->n_facts());

  for (auto &v : array_)
    v = mt();
}

size_t ZobristHash::operator()(const std::vector<int> &state) const {
  size_t seed = 0;

  for (int i=0, n=n_; i<n; ++i)
    seed ^= array_[problem_->Fact(i, state[i])];

  return seed;
}

} // namespace pplanner
