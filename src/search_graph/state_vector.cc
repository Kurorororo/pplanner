#include "search_graph/state_vector.h"

namespace pplanner {

void StateVector::ResizeClosed() {
  int new_exponent = 1;

  while ((1u << new_exponent) < 3 * n_closed_)
    ++new_exponent;

  uint32_t new_mask = (1u << new_exponent) - 1;
  std::vector<int> new_closed(1 << new_exponent, -1);
  size_t b_size = packer_->block_size();

  for (int k=0, m=closed_.size(); k<m; ++k) {
    int id = closed_[k];

    if (id != -1) {
      auto packed = states_.data() + static_cast<size_t>(id) * b_size;
      size_t i = hash_->operator()(packed) & new_mask;

      while (new_closed[i] != -1)
        i = i == (new_closed.size() - 1) ? 0 : i + 1;

      new_closed[i] = id;
    }
  }

  closed_exponent_ = new_exponent;
  closed_mask_ = new_mask;
  closed_.swap(new_closed);
}

void StateVector::Close(int node) {
  if (2 * (n_closed_ + 1) > closed_.size())
    ResizeClosed();

  size_t b_size = packer_->block_size();
  auto packed = states_.data() + static_cast<size_t>(node) * b_size;
  size_t i = hash_->operator()(packed) & closed_mask_;

  while (closed_[i] != -1)
    i = i == (closed_.size() - 1) ? 0 : i + 1;

  ++n_closed_;
  closed_[i] = node;
}

int StateVector::GetClosed(const std::vector<int> &state) const {
  packer_->Pack(state, tmp_packed_.data());
  size_t i = hash_->operator()(tmp_packed_) & closed_mask_;
  size_t b_size = packer_->block_size();

  while (closed_[i] != -1) {
    auto packed = states_.data() + static_cast<size_t>(closed_[i]) * b_size;
    bool all = true;

    for (size_t j=0; j<b_size; ++j) {
      if (tmp_packed_[j] != packed[j]) {
        all = false;
        break;
      }
    }

    if (all) return closed_[i];
    i = (i == closed_.size() - 1) ? 0 : i + 1;
  }

  return -1;
}

} // namespace pplanner
