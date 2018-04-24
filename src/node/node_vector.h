#ifndef NODE_VECTOR_H_
#define NODE_VECTOR_H_

#include <cstdint>
#include <cstring>

#include <iostream>
#include <vector>

namespace rwls {

int NodeVectorSize(size_t ram_size, size_t block_size, size_t additional);

class NodeVector {
 public:
  NodeVector() : capacity_(0), front_(0) {}

  NodeVector(int size, int block_size) : capacity_(size), front_(0) {
    std::cout << "Initial size of node bucket: " << size << std::endl;
    block_size_ = static_cast<size_t>(block_size);
    action_.reserve(size); parent_.reserve(size);
    state_.reserve(block_size * static_cast<size_t>(size));
  }

  uint32_t* GetState(int i) {
    return state_.data() + static_cast<size_t>(i) * block_size_;
  }

  const uint32_t* GetState(int i) const {
    return state_.data() + static_cast<size_t>(i) * block_size_;
  }

  int GetAction(int i) const {
    return action_[i];
  }

  int GetParent(int i) const {
    return parent_[i];
  }

  int GetFront() {
    return front_;
  }

  int GetCapacity() {
    return capacity_;
  }

  int GenerateNode(int a, int p, const uint32_t *packed) {
    int index = front_++;
    if (front_ > capacity_) Resize();
    state_.resize(front_ * block_size_);
    action_.resize(front_);
    parent_.resize(front_);
    uint32_t *s = state_.data() + static_cast<size_t>(index) * block_size_;
    memcpy(s, packed, block_size_ * sizeof(uint32_t));
    action_[index] = a;
    parent_[index] = p;

    return index;
  }

  void Resize() {
    capacity_ *= 1.1;
    std::cout << "Resized bucket size: " << capacity_ << std::endl;
    action_.reserve(capacity_);
    parent_.reserve(capacity_);
    state_.reserve(block_size_ * static_cast<size_t>(capacity_));
  }

  void InsertToAnother(int i, std::vector<int> &v) {
    int index = front_ - 1;
    if (front_ > static_cast<int>(v.capacity())) v.reserve(1.1 * v.capacity());
    v.resize(front_);
    v[index] = i;
  }

  void GenerateBits(size_t size, std::vector<uint8_t> &bits) {
    size_t front_size = static_cast<size_t>(front_) * size;
    if (front_size > bits.capacity()) bits.reserve(1.1 * bits.capacity());
    bits.resize(front_size, 0);
  }

  uint8_t* GetBits(size_t size, int i, std::vector<uint8_t> &bits) {
    return bits.data() + static_cast<size_t>(i) * size;
  }


  std::vector<int> ExtractPath(int node);

 private:
  int capacity_;
  int front_;
  size_t block_size_;
  std::vector<int> action_;
  std::vector<int> parent_;
  std::vector<uint32_t> state_;
};

} // namespace rwls

#endif // NODE_VECTOR_H_
