#ifndef DISTRIBUTED_SEARCH_GRAPH_WITH_TIMESTAMP_H_
#define DISTRIBUTED_SEARCH_GRAPH_WITH_TIMESTAMP_H_

#include <bitset>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
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

  void Expand(int i) override {
    auto now = std::chrono::system_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now - start_).count();
    long long int timestamp = static_cast<long long int>(ns);
    timestamps_.push_back(std::make_pair(timestamp, i));
  }

  void Dump() override {
    auto indices = ArgSort(timestamps_);

    std::string filename = "expanded_nodes_";
    filename += std::to_string(this->rank());
    filename += ".csv";
    std::ofstream expanded_nodes;
    expanded_nodes.open(filename, std::ios::out);
    std::vector<int> state(n_variables_);

    for (auto p : timestamps_) {
      expanded_nodes << p.first << ",";

      int node = p.second;
      const uint32_t *packed = this->PackedState(node);

      for (int i=0, n=this->state_size(); i<n; ++i)
        expanded_nodes << std::bitset<32>(packed[i]);

      expanded_nodes << std::endl;
    }
  }

 private:
  int n_variables_;
  std::chrono::system_clock::time_point start_;
  std::vector<std::pair<long long int, int> > timestamps_;
};

} // namespace pplanner

#endif // DISTRIBUTED_SEARCH_GRAPH_WITH_TIMESTAMP_H_
