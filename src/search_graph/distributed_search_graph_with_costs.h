#ifndef DISTRIBUTED_SEARCH_GRAPH_WITH_COSTS_H_
#define DISTRIBUTED_SEARCH_GRAPH_WITH_COSTS_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

template<class T>
class DistributedSearchGraphWithCosts : public T {
 public:
  DistributedSearchGraphWithCosts(std::shared_ptr<const SASPlus> problem,
                                      int closed_exponent, int n_evaluators,
                                      int rank)
    : T(problem, closed_exponent, n_evaluators, rank), problem_(problem) {}

  virtual ~DistributedSearchGraphWithCosts() {}

  virtual size_t node_size() const override {
    return T::node_size() + sizeof(int);
  }

  virtual void Reserve(size_t size) override {
    T::Reserve(size);
    costs_.reserve(size);
  }

  int Cost(int i) const override { return costs_[i]; }

  virtual void AddMoreProperties(int action, int parent_node, int parent_rank)
    override {
    T::AddMoreProperties(action, parent_node, parent_rank);

    if (parent_rank == -1)
      costs_.push_back(0);
    else if (parent_rank == this->rank())
      costs_.push_back(Cost(parent_node) + problem_->ActionCost(action));
    else
      costs_.resize(costs_.size() + 1);
  }

  virtual int GenerateNodeIfNotClosed(const unsigned char *d) override {
    int node = T::GenerateNodeIfNotClosed(d + sizeof(int));
    int cost = -1;
    memcpy(&cost, d, sizeof(int));
    if (node != -1) costs_[node] = cost;

    return node;
  }

  virtual int GenerateAndCloseNode(const unsigned char *d) override {
    int node = T::GenerateAndCloseNode(d + sizeof(int));
    int cost = -1;
    memcpy(&cost, d, sizeof(int));
    if (node != -1) costs_[node] = cost;

    return node;
  }

  virtual int GenerateNode(const unsigned char *d, std::vector<int> &values)
    override {
    int node = T::GenerateNodeIfNotClosed(d + sizeof(int));
    int cost = -1;
    memcpy(&cost, d, sizeof(int));
    costs_[node] = cost;

    return node;
  }

  virtual void BufferNode(int action, int parent_node,
                          const std::vector<int> &parent,
                          const std::vector<int> &state,
                          unsigned char *buffer) override {
    int cost = parent_node == -1 ?
      0 : Cost(parent_node) + problem_->ActionCost(action);
    memcpy(buffer, &cost, sizeof(int));
    T::BufferNode(action, parent_node, parent, state, buffer + sizeof(int));
  }

  virtual void BufferNode(int i, const unsigned char *base,
                          unsigned char *buffer) override {
    memcpy(buffer, base, sizeof(int));
    T::BufferNode(i, base + sizeof(int), buffer + sizeof(int));
  }

 private:
  std::vector<int> costs_;
  std::shared_ptr<const SASPlus> problem_;
};

} // namespace pplanner

#endif // DISTRIBUTED_SEARCH_GRAPH_WITH_COSTS_H_
