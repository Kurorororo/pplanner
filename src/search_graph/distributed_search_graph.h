#ifndef DISTRIBUTED_SEARCH_GRAPH_H_
#define DISTRIBUTED_SEARCH_GRAPH_H_

#include <memory>
#include <vector>

#include "sas_plus.h"
#include "search_graph.h"

namespace pplanner {

class DistributedSearchGraph : public SearchGraph {
 public:
  using SearchGraph::GenerateNode;
  using SearchGraph::GenerateNodeIfNotClosed;

  DistributedSearchGraph() : SearchGraph() {}

  DistributedSearchGraph(std::shared_ptr<const SASPlus> problem,
                         int closed_exponent, int rank)
    : SearchGraph(problem, closed_exponent), rank_(rank){}

  ~DistributedSearchGraph() {}

  virtual size_t NodeSize() const override {
    return sizeof(int) + SearchGraph::NodeSize();
  }

  virtual void Reserve(size_t size) override {
    SearchGraph::Reserve(size);
    parent_ranks_.reserve(size);
  }

  virtual int GenerateNode(const std::vector<int> &state, int parent,
                           int action, bool is_preferred, int parent_rank) {
    int node = GenerateNode(state, parent, action, is_preferred);
    parent_ranks_.push_back(parent_rank);

    return node;
  }

  virtual int GenerateNode(const uint32_t *packed, int parent, int action,
                           bool is_preferred, int parent_rank) {
    int node = GenerateNode(packed, parent, action, is_preferred);
    parent_ranks_.push_back(parent_rank);

    return node;
  }

  virtual int GenerateNodeIfNotClosed(const std::vector<int> &state, int parent,
                                      int action, bool is_preferred,
                                      int parent_rank) {
    int node = GenerateNodeIfNotClosed(state, parent, action, is_preferred);
    if (node != -1) parent_ranks_.push_back(parent_rank);

    return node;
  }

  virtual int GenerateNodeIfNotClosed(const uint32_t *packed, int parent,
                                      int action, bool is_preferred,
                                      int parent_rank) {
    int node = GenerateNodeIfNotClosed(packed, parent, action, is_preferred);
    if (node != -1) parent_ranks_.push_back(parent_rank);

    return node;
  }

  virtual int GenerateNodeIfNotClosed(const unsigned char *d);

  virtual int GenerateNode(const unsigned char *d, int *h);

  virtual void BufferNode(int parent, int action, const std::vector<int> &state,
                          unsigned char *buffer);

  int rank() const { return rank_; }

  int ParentRank(int i) const { return parent_ranks_[i]; }

 private:
  int rank_;
  std::vector<int> parent_ranks_;
};

} // namespace pplanner

#endif // DISTRIBUTED_SEARCH_GRAPH_H_
