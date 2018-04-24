#ifndef HDGBFS_LMC_H_
#define HDGBFS_LMC_H_

#include <vector>

#include "node/closed_list.h"
#include "node/node_vector.h"
#include "node/open_list.h"
#include "node/state_packer.h"
#include "domain/domain.h"
#include "heuristic/landmark_count.h"
#include "trie/trie.h"

namespace rwls {


class HDGBFSLmc {
 public:
  HDGBFSLmc(const Domain &domain)
      : generated(0), expanded(0), evaluated(0), deadend(0) {
    domain_ = domain;
    table_ = ConstructTable(domain);
    packer_ = StatePacker(domain.dom);

    block_size_ = packer_.PackedSize();
    lmc_.Initialize(domain);
    ac_bytes_ = lmc_.accepted_bytes;
    node_size_ = 3 * sizeof(int) + block_size_ * sizeof(uint32_t);
    node_size_ += ac_bytes_ * sizeof(uint8_t);
    int size = NodeVectorSize(block_size_, ac_bytes_);
    vec_ = NodeVector(size, block_size_);
    lm_vec_.reserve(static_cast<size_t>(size) * ac_bytes_);
    parent_rank_.reserve(size);

    closed_ = ClosedList(22, block_size_);

    tmp_accepted_.resize(ac_bytes_);
  }

  std::vector<int> operator()();

  int generated;
  int expanded;
  int evaluated;
  int deadend;

 private:
  inline int NodeVectorSize(size_t block_size, size_t ac_bytes) {
    size_t size = ((block_size + 3) * sizeof(int) + ac_bytes * sizeof(uint8_t));

    return 5000000000 / size;
  }

  void SendTermination();

  bool ReceiveTermination();

  void SendNodes();

  void ReceiveNodes();

  void BytesToNode(const unsigned char *d);

  void NodeToBytes(int to_rank, int a, int parent, const uint32_t *packed,
                   const uint8_t *accepted);

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
  std::vector<uint8_t> tmp_accepted_;

  // Data structures
  int block_size_;
  int node_size_;
  size_t ac_bytes_;
  Domain domain_;
  TrieTable table_;
  NodeVector vec_;
  std::vector<uint8_t> lm_vec_;
  OpenList<int, int> open_;
  ClosedList closed_;
  StatePacker packer_;
  LandmarkCount lmc_;
};

} // namespace rwls

#endif // HDGBFS_LMC_H_
