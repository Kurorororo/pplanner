#ifndef SUCCESSOR_GENERATOR_H_
#define SUCCESSOR_GENERATOR_H_

#include <cassert>

#include <random>
#include <utility>
#include <vector>

#include "sas_plus.h"

namespace pplanner {
class SuccessorGenerator {
 public:
  SuccessorGenerator() : problem_(nullptr), to_data_(1, 0) {}

  explicit SuccessorGenerator(std::shared_ptr<const SASPlus> problem)
      : n_variables_(problem->n_variables()),
        problem_(nullptr),
        to_data_(1, 0) {
    Init(problem);
  }

  void Init(std::shared_ptr<const SASPlus> problem);

  void Generate(const std::vector<int> &state, std::vector<int> &result) const {
    result = no_preconditions_;
    DFS(state, 0, 0, result);
  }

  int Sample(const std::vector<int> &state);

  void Dump() const;

  std::size_t to_child_size() const { return to_child_.size(); }

  const int *to_child_data() const { return to_child_.data(); }

  std::size_t to_data_size() const { return to_data_.size(); }

  const int *to_data() const { return to_data_.data(); }

  std::size_t data_size() const { return data_.size(); }

  const int *data() const { return data_.data(); }

  const int *no_preconditions() const { return no_preconditions_.data(); }

  std::size_t no_preconditions_size() const { return no_preconditions_.size(); }

 private:
  void Insert(int query, std::vector<std::pair<int, int> > &precondition,
              std::vector<int> &to_data, std::vector<std::vector<int> > &data);

  void AddQuery(int index, int query, std::vector<int> &to_data,
                std::vector<std::vector<int> > &data);

  void ConvertToData(const std::vector<int> &to_data,
                     const std::vector<std::vector<int> > &data);

  void DFS(const std::vector<int> &state, int index, int current,
           std::vector<int> &result) const;

  void DFSample(const std::vector<int> &state, int index, int current,
                unsigned int &k, int &result);

  int n_variables_;
  std::shared_ptr<const SASPlus> problem_;
  std::vector<int> to_child_;
  std::vector<int> to_data_;
  std::vector<int> data_;
  std::vector<int> no_preconditions_;
  std::mt19937 engine_;
};

}  // namespace pplanner

#endif  // SUCCESSOR_GENERATOR_H_
