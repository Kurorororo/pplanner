#include "node/state_packer.h"

#include <climits>

#include <iostream>
#include <vector>

using std::vector;

namespace rwls {

StatePacker::StatePacker(const vector<int> &dom) {
  int size = dom.size();
  block_index_.resize(size);
  vector<int> bit_index(size);
  vector<int> bit_size(size);

  int index = 0;
  int offset = 0;

  for (int i=0; i<size; ++i) {
    int n_bit = static_cast<int>(std::ceil(std::log2(dom[i])));
    assert(n_bit <= 32);
    bit_size[i] = n_bit;

    if (32 - offset < n_bit) {
      ++index;
      offset = 0;
    }

    block_index_[i] = index;
    bit_index[i] = offset;
    offset += n_bit;
  }

  block_size_ = index + 1;

  mask_.resize(size);
  shift_.resize(size);

  for (int i=0; i<size; ++i) {
    int right = 32 - bit_size[i];
    int left = right - bit_index[i];
    mask_[i] = UINT32_MAX >> right << left;
    shift_[i] = left;
  }

  v_per_b_.resize(block_size_, 0);

  for (int i=0; i<size; ++i)
    ++v_per_b_[block_index_[i]];

  std::cout << "state need " << sizeof(int) * size << " byte" << std::endl;
  std::cout << "packed state need " << sizeof(uint32_t) * block_size_
            << " byte" << std::endl;
}

void StatePacker::Pack(const State &state, uint32_t *packed) const {
  int index = 0;

  for (int i=0, n=block_size_; i<n; ++i) {
    uint32_t tmp = 0;

    for (int j=0, m=v_per_b_[i]; j<m; ++j) {
      tmp |= static_cast<uint32_t>(state[index]) << shift_[index];
      ++index;
    }

    packed[i] = tmp;
  }
}

void StatePacker::Unpack(const uint32_t *packed, State &state) const {
  for (int i=0, n=state.size(); i<n; ++i) {
    int index = block_index_[i];
    state[i] = static_cast<int>((packed[index] & mask_[i]) >> shift_[i]);
  }
}

bool BytesEqual(int block_size, const uint32_t *a, const uint32_t *b) {
  for (int i=0; i<block_size; ++i)
    if (a[i] != b[i]) return false;

  return true;
}

} // namespace rwls
