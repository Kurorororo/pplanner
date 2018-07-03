#ifndef ZOBRIST_HASH_H_
#define ZOBRIST_HASH_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

struct ZobristHash {
 public:
  ZobristHash() : n_(0), problem_(nullptr) {}

  ZobristHash(std::shared_ptr<const SASPlus> problem, uint64_t seed);

  uint64_t operator()(const std::vector<int> &state) const;

 private:
  int n_;
  std::shared_ptr<const SASPlus> problem_;
  std::vector<int> offsets_;
  std::vector<uint64_t> array_;
};

} // namespace pplanner

#endif // ZOBRIST_HASH_H_
