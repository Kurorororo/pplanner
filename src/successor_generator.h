#ifndef SUCCESSOR_GENERATOR_H_
#define SUCCESSOR_GENERATOR_H_

#include <random>
#include <utility>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class SuccessorGenerator {
 public:
  SuccessorGenerator() : problem_(nullptr) {}

  explicit SuccessorGenerator(std::shared_ptr <const SASPlus> problem)
    : problem_(nullptr) { Init(problem); }

  void Init(std::shared_ptr<const SASPlus> problem);

  void Generate(const std::vector<int> &state, std::vector<int> &result) const {
    result.clear();
    DFS(state, 0, 0, result);
  }

  int Sample(const std::vector<int> &state) {
    int result = -1;
    unsigned int k = 1;
    DFSample(state, 0, 0, k, result);

    return result;
  }

  void Dump() const;

  const int* to_child_data() const { return to_child_.data(); }

  const int* to_data() const { return to_data_.data(); }

  const std::vector<std::vector<int> >& data() const { return data_; }

 private:
  void Insert(int query, std::vector<std::pair<int, int> > &precondition);

  void AddQuery(int index, int query);

  void DFS(const std::vector<int> &state, int index, size_t current,
           std::vector<int> &result) const;

  void DFSample(const std::vector<int> &state, int index, size_t current,
                unsigned int &k, int &result);

  std::shared_ptr<const SASPlus> problem_;
  std::vector<int> to_child_;
  std::vector<int> to_data_;
  std::vector<std::vector<int> > data_;
  std::mt19937 engine_;
};

} // namespace pplanner

#endif // SUCCESSOR_GENERATOR_H_
