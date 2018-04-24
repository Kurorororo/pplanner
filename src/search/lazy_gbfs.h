#ifndef LAZY_GBFS_H_
#define LAZY_GBFS_H_

#include <vector>

#include "node/node_vector.h"
#include "node/state_packer.h"
#include "domain/domain.h"
#include "trie/trie.h"

namespace rwls {

template<class H>
class LazyGBFS {
 public:
  LazyGBFS() : generated_(0), expanded_(0), evaluated_(0), deadend_(0) {}

  std::vector<int> operator()(const Domain &domain);

  inline int generated() const { return generated_; }
  inline int expanded() const { return expanded_; }
  inline int evaluated() const { return evaluated_; }
  inline int deadend() const { return deadend_; }

 private:
  int Search(const Domain &domain, const TrieTable &table,
             const StatePacker &packer, NodeVector &vec);

  int generated_;
  int expanded_;
  int evaluated_;
  int deadend_;
};

} // namespace rwls

#include "./details/lazy_gbfs.h"

#endif // LAZY_GBFS_H_
