#ifndef SWASTAR_H_
#define SWASTAR_H_

#include <vector>

#include "node/node_vector.h"
#include "node/state_packer.h"
#include "domain/domain.h"
#include "trie/trie.h"

namespace rwls {

template<class H>
class SWAstar {
 public:
  SWAstar() : generated(0), expanded(0), evaluated(0), deadend(0) {}

  std::vector<int> operator()(int weight, const Domain &domain);

  int generated;
  int expanded;
  int evaluated;
  int deadend;

 private:
  int Search(int weight, const Domain &domain, const TrieTable &table,
             const StatePacker &packer, NodeVector &vec, std::vector<int> &step);

  std::vector<int> ExtractPath(const NodeVector &vec, int node);
};

} // namespace rwls

#include "./details/swastar.h"

#endif // SWASTAR_H_
