#include "search_graph/state_vector.h"

namespace pplanner {

using std::vector;

bool StateVector::Expand(int node, vector<int> &state) {
  Get(node, state);
  size_t block_size = packer_->block_size();
  auto packed = states_.data() + static_cast<size_t>(node) * block_size;
  size_t index = Find(state, packed);

  if (closed_[index] != -1) return false;

  Close(index, node);

  return true;
}

void StateVector::Close(size_t index, int node) {
  ++n_closed_;
  closed_[index] = node;

  if (2 * (n_closed_ + 1) > closed_.size())
    ResizeClosed();
}

void StateVector::ResizeClosed() {
  closed_exponent_ = 1;

  while ((1u << closed_exponent_) < 3 * n_closed_)
    ++closed_exponent_;

  closed_mask_ = (1u << closed_exponent_) - 1;
  std::vector<int> new_closed(1 << closed_exponent_, -1);

  for (int k=0, m=closed_.size(); k<m; ++k) {
    int id = closed_[k];

    if (id != -1) {
      Get(id, tmp_state_);
      size_t i = Hash(tmp_state_);

      while (new_closed[i] != -1)
        i = i == (new_closed.size() - 1) ? 0 : i + 1;

      new_closed[i] = id;
    }
  }

  closed_.swap(new_closed);
}

size_t StateVector::Find(const vector<int> &state, const uint32_t *packed)
  const {
  size_t hash = Hash(state);

  if (closed_[hash] == -1) return hash;

  size_t b_size = packer_->block_size();

  while (closed_[hash] != -1) {
    auto found = states_.data() + static_cast<size_t>(closed_[hash]) * b_size;
    if (BytesEqual(b_size, packed, found)) break;
    hash = (hash == closed_.size() - 1) ? 0 : hash + 1;
  }

  return hash;
}

} // namespace pplanner
