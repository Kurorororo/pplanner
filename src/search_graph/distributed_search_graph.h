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

  DistributedSearchGraph(std::shared_ptr<const SASPlus> problem,
                         int closed_exponent, int n_evaluators, int rank)
    : SearchGraph(problem, closed_exponent),
      n_evaluators_(n_evaluators),
      rank_(rank) {}

  virtual ~DistributedSearchGraph() {}

  virtual size_t node_size() const override {
    return sizeof(int) + SearchGraph::node_size();
  }

  virtual void Reserve(size_t size) override {
    SearchGraph::Reserve(size);
    parent_ranks_.reserve(size);
  }

  virtual void AddMoreProperties(int parent_rank) {
    parent_ranks_.push_back(parent_rank);
  }

  virtual int GenerateNodeIfNotClosed(const unsigned char *d);

  virtual int GenerateNode(const unsigned char *d, std::vector<int> &values);

  virtual void BufferNode(int action, int parent_node,
                          const std::vector<int> &parent,
                          const std::vector<int> &state, unsigned char *buffer);

  virtual void BufferNode(int i, const unsigned char *base,
                          unsigned char *buffer) {
    memcpy(buffer, base, node_size());
  }

  int n_evaluators() const { return n_evaluators_; }

  int rank() const { return rank_; }

  int ParentRank(int i) const { return parent_ranks_[i]; }

  int GenerateNode(int action, int parent_node, const std::vector<int> &state,
                   int parent_rank) {
    AddMoreProperties(parent_rank);

    return GenerateNode(action, parent_node, state);
  }

  int GenerateNode(int action, int parent_node, const std::vector<int> &parent,
                   const std::vector<int> &state, int parent_rank) {
    AddMoreProperties(parent_rank);

    return GenerateNode(action, parent_node, parent, state);
  }

  int GenerateNode(int action, int parent_node, uint32_t hash_value,
                   const uint32_t *packed, int parent_rank) {
    AddMoreProperties(parent_rank);

    return GenerateNode(action, parent_node, hash_value, packed);
  }

  int GenerateNodeIfNotClosed(int action, int parent_node, uint32_t hash_value,
                              const uint32_t *packed, int parent_rank);

  int GenerateNodeIfNotClosed(int action, int parent_node,
                              const std::vector<int> &state, int parent_rank);

  int GenerateNodeIfNotClosed(int action, int parent_node,
                              const std::vector<int> &parent,
                              const std::vector<int> &state, int parent_rank);

 private:
  int n_evaluators_;
  int rank_;
  std::vector<int> parent_ranks_;
};

} // namespace pplanner

#endif // DISTRIBUTED_SEARCH_GRAPH_H_
