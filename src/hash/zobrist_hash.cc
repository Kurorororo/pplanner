#include "hash/zobrist_hash.h"

#include <random>

namespace rwls {

size_t ZobristHash::operator()(const State &state) const {
  size_t seed = 0;

  for (size_t i=0, size=state.size(); i<size; ++i)
    seed ^= array[offset[i] + state[i]];

  return seed;
}

ZobristHash::ZobristHash(const Domain &domain) {
  // std::random_device rng;
  //std::mt19937_64 mt(rng());
  std::mt19937_64 mt(2886379259);
  array.resize(domain.fact_size);

  for (auto &v : array)
    v = mt();

  offset = domain.fact_offset;
}


} // namespace rwls
