#ifndef SEARCH_GRAPH_WITH_COSTS_H_
#define SEARCH_GRAPH_WITH_COSTS_H_

#include <memory>
#include <vector>

#include "sas_plus.h"
#include "search_graph.h"

namespace pplanner {

template<class T>
class SearchGraphWithCosts : public T {
 public:
  SearchGraphWithCosts(std::shared_ptr<const SASPlus> problem,
                      int closed_exponent)
    : T(problem, closed_exponent), problem_(problem) {}

  ~SearchGraphWithCosts() {}

  std::size_t node_size() const override {
    return T::node_size() + sizeof(int);
  }

  void Reserve(std::size_t size) override {
    T::Reserve(size);
    costs_.reserve(size);
  }

  void AddProperties(int action, int parent, uint32_t hash_value) override {
    T::AddProperties(action, parent, hash_value);
    int cost = parent == -1 ? 0 : Cost(parent) + problem_->ActionCost(action);
    costs_.push_back(cost);
  }

  int Cost(int i) const override { return costs_[i]; }

  int GenerateNodeIfNotClosed(int action, int parent_node, uint32_t hash_value,
                              const uint32_t *packed) override {
    std::size_t index = this->Find(hash_value, packed);
    int c = this->ClosedEntryAt(index);

    if (c != -1) {
      int cost = parent_node == -1 ?
        0 : Cost(parent_node) + problem_->ActionCost(action);

      if (cost >= Cost(c)) return -1;

      this->OpenClosedEntryAt(index);
    }

    return this->GenerateNode(action, parent_node, hash_value, packed);
  }

  int GenerateAndCloseNode(int action, int parent_node, uint32_t hash_value,
                           const uint32_t *packed) override {
    std::size_t index = this->Find(hash_value, packed);
    int c = this->ClosedEntryAt(index);

    if (c != -1) {
      int cost = parent_node == -1 ?
        0 : Cost(parent_node) + problem_->ActionCost(action);

      if (cost >= Cost(c)) return -1;
    }

    int node = this->GenerateNode(action, parent_node, hash_value, packed);
    this->Close(index, node);

    return node;
  }

  bool CloseIfNot(int node) override {
    return this->CloseIfNotInner(node, true);
  }

 private:
  std::vector<int> costs_;
  std::shared_ptr<const SASPlus> problem_;
};

} // namespace pplanner

#endif // SEARCH_GRAPH_WITH_COSTS_H_
