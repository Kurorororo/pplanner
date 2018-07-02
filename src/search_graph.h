#ifndef SEARCH_GRAPH_H_
#define SEARCH_GRAPH_H_

#include <cassert>

#include <vector>

#include "sas_plus.h"
#include "search_graph/state_vector.h"
#include "landmark/landmark_graph.h"

namespace pplanner {

class SearchGraph {
 public:
  SearchGraph() : capacity_(0), resize_factor_(1.2), states_(nullptr) {}

  SearchGraph(const SASPlus &problem, int closed_exponent)
    : capacity_(0),
      resize_factor_(1.2),
      states_(std::make_shared<StateVector>(problem, closed_exponent)) {}

  virtual ~SearchGraph() {}

  virtual void InitLandmarks(std::shared_ptr<const LandmarkGraph> graph) {}

  virtual size_t NodeSize() const {
    assert(states_ != nullptr);

    return 2 * sizeof(int) + states_->state_size();
  }

  virtual void Reserve(size_t size) {
    assert(states_ != nullptr);

    actions_.reserve(size);
    parents_.reserve(size);
    states_->Reserve(size);
    capacity_ = size;
  }

  void ReserveByRAMSize(size_t ram_size) {
    size_t size = (ram_size - states_->closed_size()) / NodeSize();
    Reserve(size);
  }

  virtual int GenerateNode(const std::vector<int> &state, int parent,
                           int action, bool is_preferred) {
    ReserveIfFull();
    int node = states_->Add(state);
    AddEdge(parent, action);

    return node;
  }

  virtual int GenerateNode(const uint32_t *packed, int parent, int action,
                           bool is_preferred) {
    ReserveIfFull();
    int node = states_->Add(packed);
    AddEdge(parent, action);

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
                                      bool is_preferred) {
    ReserveIfFull();
    int node = states_->AddIfNotClosed(state);
    if (node != -1) AddEdge(parent, action);

    return node;
  }

  virtual int GenerateNodeIfNotClosed(const uint32_t *packed, int parent,
                                      int action, bool is_preferred) {
    ReserveIfFull();
    int node = states_->AddIfNotClosed(packed);
    if (node != -1) AddEdge(parent, action);

    return node;
  }

  virtual uint8_t* Landmark(int i) { return nullptr; }

  virtual uint8_t* ParentLandmark(int i) { return nullptr; }

  int GetStateAndClosed(int i, std::vector<int> &state) const {
    return states_->GetStateAndClosed(i, state);
  }

  void PackState(const std::vector<int> &state, uint32_t *packed) const {
    return states_->PackState(state, packed);
  }

 private:
  void ReserveIfFull() {
    if (actions_.size() == capacity_)
      Reserve(capacity_ * resize_factor_);
  }

  void AddEdge(int parent, int action) {
    parents_.push_back(parent);
    actions_.push_back(action);
  }

  size_t capacity_;
  float resize_factor_;
  std::vector<int> actions_;
  std::vector<int> parents_;
  std::shared_ptr<StateVector> states_;
};

std::vector<int> ExtractPath(const SearchGraph &graph, int goal);

} // namespace pplanner

#endif // SEARCH_GRAPH_H_
