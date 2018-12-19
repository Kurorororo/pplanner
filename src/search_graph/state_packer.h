#ifndef STATE_PACKER_H_
#define STATE_PACKER_H_

#include <cstdint>

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class StatePacker {
 public:
  StatePacker(std::shared_ptr<const SASPlus> problem);

  std::size_t block_size() const { return block_size_; }

  void Pack(const std::vector<int> &state, uint32_t *packed) const;

  void Unpack(const uint32_t *packed, std::vector<int> &state) const;

 private:
  std::size_t block_size_;
  std::vector<int> block_index_;
  std::vector<int> var_per_block_;
  std::vector<int> shift_;
  std::vector<uint32_t> mask_;
};

bool BytesEqual(int block_size, const uint32_t *a, const uint32_t *b);

} // namespace pplanner

#endif // STATE_PACKER_H_
