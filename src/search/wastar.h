#ifndef WASTAR_H_
#define WASTAR_H_

#include <vector>

#include "node/node_vector.h"
#include "node/state_packer.h"
#include "domain/domain.h"
#include "trie/trie.h"

namespace rwls {

template<class H>
class WAstar {
 public:
  WAstar() : generated(0), expanded(0), evaluated(0), deadend(0) {}

  std::vector<int> operator()(int weight, const Domain &domain);

  int generated;
  int expanded;
  int evaluated;
  int deadend;

 private:
  int Search(int weight, const Domain &domain, const TrieTable &table,
             const StatePacker &packer, NodeVector &vec, std::vector<int> &g_values);

  std::vector<int> ExtractPath(const NodeVector &vec, int node);
};

} // namespace rwls

#include "./details/wastar.h"

#endif // WASTAR_H_
