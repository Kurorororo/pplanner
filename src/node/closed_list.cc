#include "node/closed_list.h"

#include <iostream>
#include <random>

namespace rwls {

ClosedList::ClosedList(int d, int block_size) {
  n_ = 0;
  d_ = d;
  block_size_ = block_size;

  size_t size = 1 << d;
  t_.resize(size, -1);

  // these can be randomized by using std::random_device.
  std::mt19937 engine(1395437481);
  std::mt19937_64 engine64(866667759);

  zz_ = engine64();

  z_.resize(block_size);

  for (auto &v : z_)
    v = static_cast<uint64_t>(engine());

  std::cout << "initial closed list size: 2^" << d_ << std::endl;
}

bool ClosedList::Contain(const NodeVector &v, const uint32_t *state) const {
  uint32_t i = Hash(state);

  while (t_[i] != -1) {
    if (BytesEqual(block_size_, v.GetState(t_[i]), state)) return true;
    i = i == t_.size() - 1 ? 0 : i + 1;
  }

  return false;
}

void ClosedList::Insert(const NodeVector &v, int id) {
  if (2 * (n_ + 1) > t_.size()) Resize(v);

  uint32_t i = Hash(v.GetState(id));

  while (t_[i] != -1)
    i = i == static_cast<uint32_t>(t_.size() - 1) ? 0 : i + 1;

  ++n_;
  t_[i] = id;
}

void ClosedList::Resize(const NodeVector &v) {
  d_ = 1;
  while ((1u << d_) < 3 * n_) ++d_;
  std::vector<int> t_new(1 << d_, -1);

  for (int k=0, m=t_.size(); k<m; ++k) {
    int id = t_[k];

    if (id != -1) {
      uint32_t i = Hash(v.GetState(id));

      while (t_new[i] != -1)
        i = i == static_cast<uint32_t>(t_new.size()) - 1 ? 0 : i + 1;

      t_new[i] = id;
    }
  }

  t_.swap(t_new);

  std::cout << "resized closed list to 2^" << d_ << std::endl;
}

void ClosedList::Dump() const {
  std::cout << "d=" << d_ << " n=" << n_ << std::endl;
  std::cout << "t:";

  for (auto v : t_)
    std::cout << " " << v;

  std::cout << std::endl;
}

uint32_t ClosedList::Hash(const uint32_t *state) const {
  uint64_t hash = 0;

  for (int i=0, n=block_size_; i<n; ++i)
    hash += z_[i] * static_cast<uint64_t>(state[i]);

  return static_cast<uint32_t>((zz_ * hash) >> (64 - d_));
}

} // namespace rwls
