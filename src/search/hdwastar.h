#ifndef HDWASTAR_H_
#define HDWASTAR_H_

#include <vector>

#include "node/closed_list.h"
#include "node/open_list.h"
#include "node/node_vector.h"
#include "node/state_packer.h"
#include "domain/domain.h"
#include "trie/trie.h"

namespace rwls {

int NodeVectorSize(int block_size) {
  return 5000000000  / ((block_size + 4) * sizeof(int));
}

template<class H>
class HDWAstar {
 public:
  HDWAstar(int weight, const Domain &domain)
      : weight_(weight), generated(0), expanded(0), evaluated(0), deadend(0) {
    domain_ = domain;
    table_ = ConstructTable(domain);
    packer_ = StatePacker(domain.dom);

    block_size_ = packer_.PackedSize();
    node_size_ = 4 * sizeof(int) + block_size_ * sizeof(uint32_t);
    int size = NodeVectorSize(block_size_);
    vec_ = NodeVector(size, block_size_);
    g_values_ = std::vector<int>(size, -1);
    parent_rank_.reserve(size);

    closed_ = ClosedList(22, block_size_);
  }

  std::vector<int> operator()();

  int generated;
  int expanded;
  int evaluated;
  int deadend;

 private:
  void SendTermination();

  bool ReceiveTermination();

  void SendNodes();

  void ReceiveNodes();

  void BytesToNode(const unsigned char *d);

  void NodeToBytes(int to_rank, int a, int parent, int g, const uint32_t *packed);

  int Search();

  std::vector<int> ExtractPath(int node);

  void Flush();

  // MPI information
  int world_size_;
  int rank_;
  std::vector<int> parent_rank_;
  std::vector<std::vector<unsigned char> > outgo_buffer_;
  std::vector<unsigned char> income_buffer_;

  // tmp information
  State tmp_state_;
  State tmp_child_;
  std::vector<uint32_t> tmp_packed_;

  // Data structures
  int weight_;
  int block_size_;
  int node_size_;
  Domain domain_;
  TrieTable table_;
  NodeVector vec_;
  std::vector<int> g_values_;
  OpenList<std::pair<int, int>, int> open_;
  ClosedList closed_;
  StatePacker packer_;
  H heuristic_;
};

} // namespace rwls

#include "./details/hdwastar.h"

#endif // HDWASTAR_H_
