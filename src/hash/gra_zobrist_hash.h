#ifndef GRA_ZOBRIST_HASH_H_
#define GRA_ZOBRIST_HASH_H_

#include <array>
#include <memory>
#include <vector>

#include "sas_plus.h"
#include "hash/distribution_hash.h"

namespace pplanner {

class GRAZobristHash : public DistributionHash {
 public:
  GRAZobristHash(std::shared_ptr<const SASPlus> problem, uint32_t seed,
                 bool greedy=true, double threashold=0.0);

  ~GRAZobristHash() {}

  uint32_t operator()(const std::vector<int> &state) const override;

  uint32_t HashByDifference(int action, uint32_t seed,
                            const std::vector<int> &parent,
                            const std::vector<int> &state) override;
 private:
  std::shared_ptr<const SASPlus> problem_;
  std::vector<std::vector<int> > cuts_;
  std::vector<bool> ignore_;
  std::vector<std::array<uint32_t, 2> > array_;
};

} // namespace pplanner

#endif // GRA_ZOBRIST_HASH_H_
