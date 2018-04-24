#ifndef STATE_PACKER_H_
#define STATE_PACKER_H_

#include <cstdint>

#include <vector>

#include "domain/domain.h"
#include "domain/state.h"

namespace rwls {

class StatePacker {
 public:
  StatePacker() {};

  StatePacker(const std::vector<int> &dom);

  int PackedSize() const { return block_size_; }

  void Pack(const State &state, uint32_t *packed) const;

  void Unpack(const uint32_t *packed, State &state) const;

 private:
  size_t block_size_;
  std::vector<int> block_index_;
  std::vector<int> v_per_b_;
  std::vector<int> shift_;
  std::vector<uint32_t> mask_;
};

bool BytesEqual(int block_size, const uint32_t *a, const uint32_t *b);

} // namespace rwls

#endif // STATE_PACKER_H_
