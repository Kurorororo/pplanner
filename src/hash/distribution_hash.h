#ifndef DISTRIBUTION_HASH_H_
#define DISTRIBUTION_HASH_H_

#include <cstdint>

#include <vector>

namespace pplanner {

class DistributionHash {
 public:
  virtual ~DistributionHash() = 0;

  virtual uint32_t operator()(const std::vector<int> &state) const = 0;

  virtual uint32_t HashByDifference(int action, uint32_t seed,
                                    const std::vector<int> &parent,
                                    const std::vector<int> &state) = 0;
};

} // namespace pplanner

#endif // DISTRIBUTION_HASH_H_
