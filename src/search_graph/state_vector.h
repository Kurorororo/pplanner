#ifndef STATE_VECTOR_H_
#define STATE_VECTOR_H_

#include <cstdint>
#include <cstring>

#include <iostream>
#include <vector>

#include "hash/array_hash.h"
#include "search_graph/state_packer.h"

namespace pplanner {

class StateVector {
 public:
  StateVector()
    : n_closed_(0),
      closed_exponent_(0),
      closed_mask_(0),
      packer_(nullptr) {}

  StateVector(const SASPlus &problem, int closed_exponent=22)
    : n_closed_(0),
      closed_exponent_(closed_exponent),
      closed_mask_((1u << closed_exponent) - 1),
      packer_(std::make_shared<StatePacker>(problem)) {
    InitClosed();
  }

  size_t size() const { return states_.size() / packer_->block_size(); }

  size_t state_size() const { return packer_->block_size() * sizeof(uint32_t); }

  void Reserve(size_t size) {
    assert(packer_ != nullptr);

    states_.reserve(packer_->block_size() * size);
  }

  int Add(const std::vector<int> &state) {
    size_t old_size = states_.size();
    size_t b_size = packer_->block_size();
    states_.resize(old_size + b_size);
    packer_->Pack(state, states_.data() + old_size);

    return static_cast<int>(old_size / b_size);
  }

  void Get(int i, std::vector<int> &state) const {
    size_t index = static_cast<size_t>(i) * packer_->block_size();
    packer_->Unpack(states_.data() + index, state);
  }

  size_t closed_size() const { return closed_.size() * sizeof(int); }

  void Close(int node);

  int GetClosed(const std::vector<int> &state) const {
    packer_->Pack(state, tmp_packed_.data());

    return GetClosedFromPacked(tmp_packed_.data());
  }

  int AddIfNotClosed(const std::vector<int> &state);

  int GetStateAndClosed(int i, std::vector<int> &state) const;

 private:
  void InitClosed() {
    assert(packer_ != nullptr);

    size_t size = 1 << closed_exponent_;
    closed_.resize(size, -1);
    size_t b_size = packer_->block_size();
    tmp_packed_.resize(b_size, 0);
    hash_ = std::make_shared<ArrayHash<uint32_t> >(b_size);
  }

  void ResizeClosed();

  int GetClosedFromPacked(const uint32_t *ptr) const;

  size_t n_closed_;
  int closed_exponent_;
  uint32_t closed_mask_;
  std::vector<int> closed_;
  std::vector<uint32_t> states_;
  mutable std::vector<uint32_t> tmp_packed_;
  std::shared_ptr<StatePacker> packer_;
  std::shared_ptr<ArrayHash<uint32_t> > hash_;
};

} // namespace pplanner

#endif // STATE_VECTOR_H_
