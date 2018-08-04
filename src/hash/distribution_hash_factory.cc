#include "hash/distribution_hash_factory.h"

#include "hash/zobrist_hash.h"
#include "hash/gra_zobrist_hash.h"

namespace pplanner {

std::shared_ptr<DistributionHash> DistributionHashFactory(
    std::shared_ptr<const SASPlus> problem,
    uint32_t seed,
    const std::string &abstraction) {
  if (abstraction == "gra")
    return std::make_shared<GRAZobristHash>(problem, seed);

  return std::make_shared<ZobristHash>(problem, seed);
}

} // namespace pplanner
