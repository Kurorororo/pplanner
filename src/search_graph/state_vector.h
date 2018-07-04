#ifndef STATE_VECTOR_H_
#define STATE_VECTOR_H_

#include <cstdint>
#include <cstring>

#include <memory>
#include <random>
#include <vector>

#include "hash/zobrist_hash.h"
#include "search_graph/state_packer.h"

namespace pplanner {

class StateVector {
 public:
  StateVector(std::shared_ptr<const SASPlus> problem, int closed_exponent=22)
    : n_closed_(0),
      closed_exponent_(closed_exponent),
      closed_mask_((1u << closed_exponent) - 1),
      closed_(1 << closed_exponent, -1),
      packer_(std::make_shared<StatePacker>(problem)),
      tmp_state_(problem->n_variables()) {
    std::random_device rnd;
    hash_ = std::make_shared<ZobristHash>(problem, rnd());
    tmp_packed_.resize(packer_->block_size(), 0);
  }

  size_t size() const { return states_.size() / packer_->block_size(); }

  size_t state_size() const { return packer_->block_size() * sizeof(uint32_t); }

  size_t closed_size() const { return closed_.size() * sizeof(int); }

  void Reserve(size_t size) {
    states_.reserve(packer_->block_size() * size);
  }

  void Pack(const std::vector<int> &state, uint32_t *packed) const {
    packer_->Pack(state, packed);
  }

  void Unpack(const uint32_t *packed, std::vector<int> &state) const {
    packer_->Unpack(packed, state);
  }

  int Add(const std::vector<int> &state) {
    size_t block_size = packer_->block_size();
    size_t old_size = states_.size();
    states_.resize(old_size + block_size);
    Pack(state, states_.data() + old_size);

    return static_cast<int>(old_size / block_size);
  }

  int Add(const uint32_t *packed) {
    size_t block_size = packer_->block_size();
    size_t old_size = states_.size();
    states_.resize(old_size + block_size);
    memcpy(states_.data() + old_size, packed, block_size * sizeof(uint32_t));

    return static_cast<int>(old_size / block_size);
  }

  int AddIfNotClosed(const std::vector<int> &state) {
    Pack(state, tmp_packed_.data());

    return AddIfNotClosed(state, tmp_packed_.data());
  }

  int AddIfNotClosed(const uint32_t *packed) {
    Unpack(packed, tmp_state_);

    return AddIfNotClosed(tmp_state_, packed);
  }

  int AddIfNotClosed(const std::vector<int> &state, const uint32_t *packed) {
    if (closed_[Find(state, packed)] != -1) return -1;

    return Add(packed);
  }

  void Get(int i, std::vector<int> &state) const {
    size_t block_size = packer_->block_size();
    auto packed = states_.data() + static_cast<size_t>(i) * block_size;
    Unpack(packed, state);
  }

  int GetClosed(const std::vector<int> &state) const {
    return closed_[Find(state)];
  }

  void Close(int node) {
    Get(node, tmp_state_);
    size_t index = Find(tmp_state_);

    Close(index, node);
  }

  bool Expand(int node, std::vector<int> &state);

 private:
  size_t Hash(const std::vector<int> &state) const {
    return hash_->operator()(state) & closed_mask_;
  }

  void Close(size_t index, int node);

  void ResizeClosed();

  size_t Find(const std::vector<int> &state) const {
    Pack(state, tmp_packed_.data());

    return Find(state, tmp_packed_.data());
  }

  size_t Find(const std::vector<int> &state, const uint32_t *packed) const;

  size_t n_closed_;
  int closed_exponent_;
  uint64_t closed_mask_;
  std::vector<int> closed_;
  std::vector<uint32_t> states_;
  std::shared_ptr<StatePacker> packer_;
  std::shared_ptr<ZobristHash> hash_;
  mutable std::vector<int> tmp_state_;
  mutable std::vector<uint32_t> tmp_packed_;
};

} // namespace pplanner

#endif // STATE_VECTOR_H_
