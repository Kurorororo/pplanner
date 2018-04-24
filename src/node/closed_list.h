#ifndef CLOSED_LIST_H_
#define CLOSED_LIST_H_

#include <cstdint>

#include <vector>

#include "node/node_vector.h"
#include "node/state_packer.h"

namespace rwls {

class ClosedList {
 public:
  ClosedList() {};

  ClosedList(int d, int block_size);

  bool Contain(const NodeVector &v, const uint32_t *state) const;

  void Insert(const NodeVector &v, int id);

  void Resize(const NodeVector &v);

  uint32_t Hash(const uint32_t *state) const;

  void Dump() const;

 private:
  size_t n_;
  int d_;
  int block_size_;
  uint64_t zz_;
  std::vector<int> t_;
  std::vector<uint64_t> z_;
};

} // namespace rwls

#endif // CLOSED_LIST_H_
