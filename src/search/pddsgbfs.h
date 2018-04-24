#ifndef PDDSGBFS_H_
#define PDDSGBFS_H_

#include <vector>

#include "node/closed_list.h"
#include "node/node_vector.h"
#include "node/state_packer.h"
#include "domain/domain.h"
#include "node/open_list.h"
#include "trie/trie.h"

namespace rwls {

int NodeVectorSize(int block_size) {
  return 5000000000  / ((block_size + 3) * sizeof(int));
}

template<class H>
class PDDSGBFS {
 public:
  PDDSGBFS(const Domain &domain)
      : generated(0), expanded(0), evaluated(0), deadend(0) {
    domain_ = domain;
    table_ = ConstructTable(domain);
    packer_ = StatePacker(domain.dom);

    block_size_ = packer_.PackedSize();
    node_size_ = 3 * sizeof(int) + block_size_ * sizeof(uint32_t);
    int size = NodeVectorSize(block_size_);
    vec_ = NodeVector(size, block_size_);
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

  void NodeToBytes(int to_rank, int a, int parent, const uint32_t *packed);

  void SendDD();

  void ReceiveDD();

  bool BytesDD(const unsigned char *d);

  int Init();

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
  int block_size_;
  int node_size_;
  Domain domain_;
  TrieTable table_;
  NodeVector vec_;
  OpenList<int, int> open_;
  ClosedList closed_;
  StatePacker packer_;
  H heuristic_;
};

} // namespace rwls

#include "./details/pddsgbfs.h"

#endif // PDDSGBFS_H_
