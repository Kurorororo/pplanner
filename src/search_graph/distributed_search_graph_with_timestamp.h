#ifndef DISTRIBUTED_SEARCH_GRAPH_WITH_TIMESTAMP_H_
#define DISTRIBUTED_SEARCH_GRAPH_WITH_TIMESTAMP_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "sas_plus.h"
#include "utils/arg_sort.h"

namespace pplanner {

template<class T>
class DistributedSearchGraphWithTimestamp : public T {
 public:
  DistributedSearchGraphWithTimestamp(std::shared_ptr<const SASPlus> problem,
                                      int closed_exponent, int n_evaluators,
                                      int rank)
    : T(problem, closed_exponent, n_evaluators, rank),
      n_variables_(problem->n_variables()) {
    start_ = std::chrono::system_clock::now();
  }

  ~DistributedSearchGraphWithTimestamp() {}

  void Expand(int i, std::vector<int> &state) override {
    T::Expand(i, state);
    ids_.push_back(i);
    auto now = std::chrono::system_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now - start_).count();
    long long int timestamp = static_cast<long long int>(ns);
    timestamps_.push_back(timestamp);
  }

  void SetH(int i, int h) override {
    if (hs_.size() <= i)
      hs_.resize(i + 1);

    hs_[i] = h;
  }

  void Dump() override {
    auto indices = ArgSort(timestamps_);

    std::string filename = "expanded_nodes_";
    filename += std::to_string(this->rank());
    filename += ".csv";
    std::ofstream expanded_nodes;
    expanded_nodes.open(filename, std::ios::out);

    if (this->rank() == 0) {
      expanded_nodes << "rank_id,parent_rank_id,node_id,parent_node_id,h,timestamp";

      for (int i=0; i<n_variables_; ++i)
        expanded_nodes << ",v" << i;

      expanded_nodes << std::endl;
    }

    std::vector<int> state(n_variables_);

    for (int i=0, n=ids_.size(); i<n; ++i) {
      int node = ids_[i];
      expanded_nodes << this->rank() << "," << this->ParentRank(node) << ",";
      expanded_nodes << node << "," << this->Parent(node) << ",";
      expanded_nodes << hs_[node] << "," << timestamps_[i];
      this->State(node, state);

      for (int i=0; i<n_variables_; ++i)
        expanded_nodes << "," << state[i];

      expanded_nodes << std::endl;
    }
  }

 private:
  int n_variables_;
  std::chrono::system_clock::time_point start_;
  std::vector<int> ids_;
  std::vector<int> hs_;
  std::vector<long long int> timestamps_;
};

} // namespace pplanner

#endif // DISTRIBUTED_SEARCH_GRAPH_WITH_TIMESTAMP_H_
