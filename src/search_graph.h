#ifndef SEARCH_GRAPH_H_
#define SEARCH_GRAPH_H_

#include <cassert>

#include <vector>

#include "search_graph/state_vector.h"

namespace pplanner {

class SearchGraph {
 public:
  SearchGraph()
    : states_(nullptr) {}

  SearchGraph(const SASPlus &problem, int closed_exponent)
    : states_(std::make_shared<StateVector>(problem, closed_exponent)) {}

  virtual ~SearchGraph() {}

  virtual size_t NodeSize() const {
    assert(states_ != nullptr);

    return 2 * sizeof(int) + states_->state_size();
  }

  virtual void Reserve(size_t size) {
    assert(states_ != nullptr);

    actions_.reserve(size);
    parents_.reserve(size);
    states_->Reserve(size);
  }

  void ReserveByRAMSize(size_t ram_size) {
    size_t size = (ram_size - states_->closed_size()) / NodeSize();
    Reserve(size);
  }

  virtual int GenerateNode(const std::vector<int> &state, int parent,
                           int action, bool is_preferred) {
    int node = states_->Add(state);
    parents_.push_back(parent);
    actions_.push_back(action);

    return node;
  }

  int Action(int i) const { return actions_[i]; }

  int Parent(int i) const { return parents_[i]; }

  void Close(int node) { states_->Close(node); }

  int GetClosed(const std::vector<int> &state) const {
    return states_->GetClosed(state);
  }

  void State(int i, std::vector<int> &state) const {
    assert(states_ != nullptr);

    states_->Get(i, state);
  }

  virtual int GenerateNodeIfNotClosed(const std::vector<int> &state,
                                      int parent, int action,
                                      bool is_preferred);

  int GetStateAndClosed(int i, std::vector<int> &state) const {
    return states_->GetStateAndClosed(i, state);
  }

 private:
  std::vector<int> actions_;
  std::vector<int> parents_;
  std::shared_ptr<StateVector> states_;
};

std::vector<int> ExtractPath(const SearchGraph &graph, int goal);

} // namespace pplanner

#endif // SEARCH_GRAPH_H_
