#include "search_graph/state_vector.h"

namespace pplanner {

using std::vector;

bool StateVector::CloseIfNot(int node, vector<int> &state) {
  Get(node, state);
  size_t index = Find(state);

  if (closed_[index] != -1) return false;

  Close(index, closed_[index]);

  return true;
}

void StateVector::Close(size_t index, int node) {
  if (2 * (n_closed_ + 1) > closed_.size())
    ResizeClosed();

  ++n_closed_;
  closed_[index] = node;
}

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
      Get(id, tmp_state_);
      size_t i = Hash(tmp_state_);

      while (new_closed[i] != -1)
        i = i == (new_closed.size() - 1) ? 0 : i + 1;

      new_closed[i] = id;
    }
  }

  closed_exponent_ = new_exponent;
  closed_mask_ = new_mask;
  closed_.swap(new_closed);
}

size_t StateVector::Find(const vector<int> &state) const {
  size_t i = Hash(state);

  if (closed_[i] == -1) return i;

  PackState(state, tmp_packed_.data());
  size_t b_size = packer_->block_size();

  while (closed_[i] != -1) {
    auto packed = states_.data() + static_cast<size_t>(closed_[i]) * b_size;
    bool all = true;

    for (size_t j=0; j<b_size; ++j) {
      if (tmp_packed[j] != packed[j]) {
        all = false;
        break;
      }
    }

    if (all) break;
    i = (i == closed_.size() - 1) ? 0 : i + 1;
  }

  return i;
}

} // namespace pplanner
