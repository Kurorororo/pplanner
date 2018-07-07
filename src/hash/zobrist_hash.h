#ifndef ZOBRIST_HASH_H_
#define ZOBRIST_HASH_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

struct ZobristHash {
 public:
  ZobristHash(std::shared_ptr<const SASPlus> problem, uint32_t seed);

  uint32_t operator()(const std::vector<int> &state) const;

  uint32_t HashByDifference(int action, uint32_t seed,
                            const std::vector<int> &parent,
                            const std::vector<int> &state);
 private:
  int n_;
  std::shared_ptr<const SASPlus> problem_;
  std::vector<int> offsets_;
  std::vector<uint32_t> array_;
};

} // namespace pplanner

#endif // ZOBRIST_HASH_H_
