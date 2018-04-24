#ifndef ZOBRIST_HASH_H_
#define ZOBRIST_HASH_H_

#include <vector>

#include "domain/domain.h"
#include "domain/state.h"

namespace rwls {

struct ZobristHash {
  ZobristHash(const Domain &domain);

  size_t operator()(const State &state) const;

  std::vector<int> offset;
  std::vector<size_t> array;
};

} // namespace rwls

#endif // ZOBRIST_HASH_H_
