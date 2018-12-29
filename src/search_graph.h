#ifndef SEARCH_GRAPH_H_
#define SEARCH_GRAPH_H_

#include <cstdint>
#include <cstring>

#include <memory>
#include <random>
#include <vector>
#include <iostream>

#include "landmark/landmark_graph.h"
#include "hash/zobrist_hash.h"
#include "search_graph/state_packer.h"

namespace pplanner {

class SearchGraph {
 public:
  SearchGraph(std::shared_ptr<const SASPlus> problem, int closed_exponent=22)
    : capacity_(0),
      resize_factor_(1.2),
      n_closed_(0),
      closed_exponent_(closed_exponent),
      closed_mask_((1u << closed_exponent) - 1),
      closed_(1 << closed_exponent, -1),
      packer_(std::make_shared<StatePacker>(problem)) {
    hash_ = std::make_shared<ZobristHash>(problem, 4166245435);
  }

  virtual ~SearchGraph() {}

  virtual void InitLandmarks(std::shared_ptr<const LandmarkGraph> graph) {}

  virtual std::size_t node_size() const {
    std::size_t state_size = packer_->block_size() * sizeof(uint32_t);

    return state_size + 2 * sizeof(int) + sizeof(uint32_t);
  }

  virtual void Reserve(std::size_t size) {
    states_.reserve(packer_->block_size() * size);
    actions_.reserve(size);
    parents_.reserve(size);
    hash_values_.reserve(size);
    capacity_ = size;
  }

  virtual void AddProperties(int action, int parent, uint32_t hash_value) {
    actions_.push_back(action);
    parents_.push_back(parent);
    hash_values_.push_back(hash_value);
  }

  virtual uint8_t* Landmark(int i) { return nullptr; }

  virtual uint8_t* ParentLandmark(int i) { return nullptr; }

  virtual void Expand(int i, std::vector<int> &state) { State(i, state); }

  virtual int GenerateNodeIfNotClosed(int action, int parent_node,
                                      uint32_t hash_value,
                                      const uint32_t *packed);

  virtual int GenerateAndCloseNode(int action, int parent_node,
                                   uint32_t hash_value,
                                   const uint32_t *packed);

  virtual bool CloseIfNot(int node) { return CloseIfNotInner(node, false); }

  virtual void SetH(int i, int h) {}

  virtual int Cost(int i) const { return -1; }

  virtual void Dump() {}

  std::size_t capacity() const { return capacity_; }

  std::size_t size() const { return hash_values_.size(); }

  std::size_t closed_size() const { return closed_.size() * sizeof(int); }

  std::size_t state_size() const {
    return packer_->block_size() * sizeof(uint32_t);
  }

  std::size_t ReserveByRAMSize(std::size_t ram_size) {
    std::size_t size = (ram_size - closed_size()) / node_size();
    Reserve(size);

    return size;
  }

  int Action(int i) const { return actions_[i]; }

  int Parent(int i) const { return parents_[i]; }

  uint32_t HashValue(int i) const { return hash_values_[i]; }

  void State(int i, std::vector<int> &state) const {
    std::size_t block_size = packer_->block_size();
    auto packed = states_.data() + static_cast<std::size_t>(i) * block_size;
    Unpack(packed, state);
  }

  const uint32_t* PackedState(int i) const {
    std::size_t block_size = packer_->block_size();
    return states_.data() + static_cast<std::size_t>(i) * block_size;
  }

  int GenerateNode(int action, int parent_node, const std::vector<int> &state) {
    ReserveIfFull();

    auto hash_value = hash_->operator()(state);
    AddProperties(action, parent_node, hash_value);
    AddPacked(state);

    return size() - 1;
  }

  int GenerateNode(int action, int parent_node, const std::vector<int> &parent,
                   const std::vector<int> &state) {
    ReserveIfFull();

    uint32_t hash_value = HashByDifference(action, parent_node, parent, state);
    AddProperties(action, parent_node, hash_value);
    AddPacked(state);

    return size() - 1;
  }

  int GenerateNode(int action, int parent_node, uint32_t hash_value,
                   const uint32_t *packed) {
    ReserveIfFull();

    AddProperties(action, parent_node, hash_value);
    AddPacked(packed);

    return size() - 1;
  }

  int GenerateNodeIfNotClosed(int action, int parent_node,
                              const std::vector<int> &state) {
    thread_local std::vector<uint32_t> tmp_packed(
        state_size() / sizeof(uint32_t));

    uint32_t hash_value = hash_->operator()(state);
    Pack(state, tmp_packed.data());

    return GenerateNodeIfNotClosed(
        action, parent_node, hash_value, tmp_packed.data());
  }

  int GenerateNodeIfNotClosed(int action, int parent_node,
                              const std::vector<int> &parent,
                              const std::vector<int> &state) {
    thread_local std::vector<uint32_t> tmp_packed(
        state_size() / sizeof(uint32_t));

    uint32_t hash_value = HashByDifference(action, parent_node, parent, state);
    Pack(state, tmp_packed.data());

    return GenerateNodeIfNotClosed(
        action, parent_node, hash_value, tmp_packed.data());
  }

  int GenerateAndCloseNode(int action, int parent_node,
                           const std::vector<int> &state) {
    thread_local std::vector<uint32_t> tmp_packed(
        state_size() / sizeof(uint32_t));

    uint32_t hash_value = hash_->operator()(state);
    Pack(state, tmp_packed.data());

    return GenerateAndCloseNode(
        action, parent_node, hash_value, tmp_packed.data());
  }

  int GenerateAndCloseNode(int action, int parent_node,
                           const std::vector<int> &parent,
                           const std::vector<int> &state) {
    thread_local std::vector<uint32_t> tmp_packed(
        state_size() / sizeof(uint32_t));

    uint32_t hash_value = HashByDifference(action, parent_node, parent, state);
    Pack(state, tmp_packed.data());

    return GenerateAndCloseNode(
        action, parent_node, hash_value, tmp_packed.data());
  }

  int GetClosed(int i) const { return closed_[Find(i)]; }

  int GetClosed(uint32_t hash_value, const uint32_t *packed) const {
    return closed_[Find(hash_value, packed)];
  }

  int GetClosed(int action, int parent_node, const std::vector<int> &parent,
                const std::vector<int> &state, uint32_t *packed,
                uint32_t *hash_value) {
    *hash_value = HashByDifference(action, parent_node, parent, state);
    packer_->Pack(state, packed);

    return GetClosed(*hash_value, packed);
  }

  bool CloseIfNotInner(int node, bool reopen_closed);

  void Close(int i) { Close(Find(i), i); }

  uint32_t Hash(const std::vector<int> &state) const {
    return hash_->operator()(state);
  }

  uint32_t HashByDifference(int action, int parent_node,
                            const std::vector<int> &parent,
                            const std::vector<int> &state) const{
    auto seed = HashValue(parent_node);

    return hash_->HashByDifference(action, seed, parent, state);
  }

  void Pack(const std::vector<int> &state, uint32_t *packed) const {
    packer_->Pack(state, packed);
  }

  void Unpack(const uint32_t *packed, std::vector<int> &state) const {
    packer_->Unpack(packed, state);
  }

  std::size_t block_size() const { return packer_->block_size(); }

  std::size_t Find(int i) const {
    uint32_t hash_value = HashValue(i);
    std::size_t block_size = packer_->block_size();
    auto packed = states_.data() + static_cast<std::size_t>(i) * block_size;

    return Find(hash_value, packed);
  }

  std::size_t Find(uint32_t hash_value, const uint32_t *packed) const;

  void Close(std::size_t index, int node);

  int ClosedEntryAt(std::size_t i) const { return closed_[i]; }

  void OpenClosedEntryAt(std::size_t i) { closed_[i] = -1; }

 private:
  void ReserveIfFull() {
    if (actions_.size() == capacity_)
      Reserve(capacity_ * resize_factor_);
  }

  void AddPacked(const std::vector<int> &state) {
    std::size_t old_size = states_.size();
    states_.resize(old_size + packer_->block_size());
    Pack(state, states_.data() + old_size);
  }

  void AddPacked(const uint32_t *packed) {
    std::size_t block_size = packer_->block_size();
    std::size_t old_size = states_.size();
    states_.resize(old_size + block_size);
    memcpy(states_.data() + old_size, packed, block_size * sizeof(uint32_t));
  }

  void ResizeClosed();

  std::size_t capacity_;
  float resize_factor_;
  std::size_t n_closed_;
  int closed_exponent_;
  uint32_t closed_mask_;
  std::vector<int> closed_;
  std::vector<int> actions_;
  std::vector<int> parents_;
  std::vector<uint32_t> states_;
  std::vector<uint32_t> hash_values_;
  std::shared_ptr<StatePacker> packer_;
  std::shared_ptr<ZobristHash> hash_;
};

std::vector<int> ExtractPath(std::shared_ptr<const SearchGraph> graph,
                             int goal);

} // namespace pplanner

#endif // SEARCH_GRAPH_H_
