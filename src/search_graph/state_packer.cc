#include "search_graph/state_packer.h"

#include <cmath>

#include <iostream>

using std::vector;

namespace pplanner {

StatePacker::StatePacker(std::shared_ptr<const SASPlus> problem) {
  int size = problem->n_variables();
  block_index_.resize(size);
  vector<int> bit_index(size);
  vector<int> bit_size(size);

  int index = 0;
  int offset = 0;

  for (int i=0; i<size; ++i) {
    int n_bit = static_cast<int>(std::ceil(std::log2(problem->VarRange(i))));
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

  var_per_block_.resize(block_size_, 0);

  for (int i=0; i<size; ++i)
    ++var_per_block_[block_index_[i]];
}

void StatePacker::Pack(const vector<int> &state, uint32_t *packed) const {
  int index = 0;

  for (int i=0, n=block_size_; i<n; ++i) {
    uint32_t tmp = 0;

    for (int j=0, m=var_per_block_[i]; j<m; ++j) {
      tmp |= static_cast<uint32_t>(state[index]) << shift_[index];
      ++index;
    }

    packed[i] = tmp;
  }
}

void StatePacker::Pack(const int *state, uint32_t *packed) const {
  int index = 0;

  for (int i=0, n=block_size_; i<n; ++i) {
    uint32_t tmp = 0;

    for (int j=0, m=var_per_block_[i]; j<m; ++j) {
      tmp |= static_cast<uint32_t>(state[index]) << shift_[index];
      ++index;
    }

    packed[i] = tmp;
  }
}

void StatePacker::Unpack(const uint32_t *packed, vector<int> &state) const {
  for (int i=0, n=state.size(); i<n; ++i) {
    int index = block_index_[i];
    state[i] = static_cast<int>((packed[index] & mask_[i]) >> shift_[i]);
  }
}

void StatePacker::Unpack(const uint32_t *packed, int *state) const {
  for (int i=0, n=block_index_.size(); i<n; ++i) {
    int index = block_index_[i];
    state[i] = static_cast<int>((packed[index] & mask_[i]) >> shift_[i]);
  }
}

bool BytesEqual(int block_size, const uint32_t *a, const uint32_t *b) {
  for (int i=0; i<block_size; ++i)
    if (a[i] != b[i]) return false;

  return true;
}

} // namespace pplanner
