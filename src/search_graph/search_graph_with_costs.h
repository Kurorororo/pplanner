#ifndef SEARCH_GRAPH_WITH_COSTS_H_
#define SEARCH_GRAPH_WITH_COSTS_H_

#include <cstdint>

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

  size_t node_size() const override {
    return T::node_size() + sizeof(int);
  }

  void Reserve(size_t size) override {
    T::Reserve(size);
    costs_.reserve(size);
  }

  void AddProperties(int action, int parent, uint32_t hash_value) override {
    T::AddProperties(action, parent, hash_value);
    int parent_cost = parent == -1 ? 0 : Cost(parent);
    costs_.push_back(parent_cost + problem_->ActionCost(action));
  }

  int Cost(int i) const override { return costs_[i]; }

 private:
  std::vector<int> costs_;
  std::shared_ptr<const SASPlus> problem_;
};

} // namespace pplanner

#endif // SEARCH_GRAPH_WITH_COSTS_H_
