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

  void Pack(const int *state, uint32_t *packed) const;

  void Unpack(const uint32_t *packed, std::vector<int> &state) const;

  void Unpack(const uint32_t *packed, int *state) const;

  const int *block_index() const { return block_index_.data(); }

  std::size_t block_index_size() const { return block_index_.size(); }

  const int *var_per_block() const { return var_per_block_.data(); }

  std::size_t var_per_block_size() const { return var_per_block_.size(); }

  const int *shift() const { return shift_.data(); }

  std::size_t shift_size() const { return shift_.size(); }

  const uint32_t *mask() const { return mask_.data(); }

  std::size_t mask_size() const { return mask_.size(); }

  void Dump() const;

 private:
  std::size_t block_size_;
  std::vector<int> block_index_;
  std::vector<int> var_per_block_;
  std::vector<int> shift_;
  std::vector<uint32_t> mask_;
};

bool BytesEqual(int block_size, const uint32_t *a, const uint32_t *b);

}  // namespace pplanner

#endif  // STATE_PACKER_H_
