#ifndef SEARCH_GRAPH_WITH_TIMESTAMP_H_
#define SEARCH_GRAPH_WITH_TIMESTAMP_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <vector>

#include "sas_plus.h"
#include "utils/arg_sort.h"

namespace pplanner {

template <class T>
class SearchGraphWithTimestamp : public T {
 public:
  SearchGraphWithTimestamp(std::shared_ptr<const SASPlus> problem,
                           int closed_exponent = 22)
      : T(problem, closed_exponent),
        n_variables_(problem->n_variables()),
        start_(std::chrono::system_clock::now()) {}

  ~SearchGraphWithTimestamp() {}

  void Expand(int i, std::vector<int> &state) override {
    T::Expand(i, state);
    ids_.push_back(i);
    auto now = std::chrono::system_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_)
                  .count();
    long long int timestamp = static_cast<long long int>(ns);
    timestamps_.push_back(timestamp);
  }

  int GenerateNodeIfNotClosed(int action, int parent_node, uint32_t hash_value,
                              const uint32_t *packed) override {
    int node =
        T::GenerateNodeIfNotClosed(action, parent_node, hash_value, packed);

    if (node == -1) {
      std::size_t index = T::Find(hash_value, packed);
      int child = T::ClosedEntryAt(index);
      closed_parent_.push_back(std::make_pair(child, parent_node));
    }

    return node;
  }

  virtual bool CloseIfNot(int node) override {
    bool result = T::CloseIfNot(node);

    if (!result) {
      int child = T::GetClosed(node);
      closed_parent_.push_back(std::make_pair(child, T::Parent(node)));

      if (hs_[node] != hs_[child]) {
        std::cout << "different h value: " << node << " and " << child
                  << std::endl;
        std::cout << hs_[node] << " vs. " << hs_[child] << std::endl;
      }
    }

    return result;
  }

  void SetH(int i, int h) override {
    if (static_cast<int>(hs_.size()) <= i) hs_.resize(i + 1);

    hs_[i] = h;
  }

  void MarkGoal(int i) override { goals_.insert(i); }

  void Dump() override {
    auto indices = ArgSort(timestamps_);

    std::ofstream expanded_nodes;
    expanded_nodes.open("expanded_nodes.csv", std::ios::out);

    expanded_nodes << "node_id,parent_node_id,h,action,is_goal,timestamp";

    for (int i = 0; i < n_variables_; ++i) expanded_nodes << ",v" << i;
    expanded_nodes << std::endl;

    std::vector<int> state(n_variables_);

    for (int i = 0, n = ids_.size(); i < n; ++i) {
      int node = ids_[i];
      int is_goal = goals_.find(node) == goals_.end() ? 0 : 1;

      expanded_nodes << node << "," << this->Parent(node) << ",";
      expanded_nodes << hs_[node] << "," << this->Action(node) << "," << is_goal
                     << "," << timestamps_[i];

      this->State(node, state);

      for (int j = 0; j < n_variables_; ++j) expanded_nodes << "," << state[j];

      expanded_nodes << std::endl;
    }

    for (auto p : closed_parent_) {
      int node = p.first;
      int is_goal = goals_.find(node) == goals_.end() ? 0 : 1;
      expanded_nodes << node << "," << p.second << "," << hs_[node] << ","
                     << this->Action(node) << "," << is_goal;
      expanded_nodes << ",9999999999999999999";

      this->State(node, state);

      for (int i = 0; i < n_variables_; ++i) expanded_nodes << "," << state[i];

      expanded_nodes << std::endl;
    }
  }

 private:
  int n_variables_;
  std::chrono::system_clock::time_point start_;
  std::vector<int> ids_;
  std::vector<int> hs_;
  std::vector<long long int> timestamps_;
  std::vector<std::pair<int, int> > closed_parent_;
  std::unordered_set<int> goals_;
};

}  // namespace pplanner

#endif  // SEARCH_GRAPH_WITH_TIMESTAMP_H_
