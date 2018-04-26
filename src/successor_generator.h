#ifndef SUCCESSOR_GENERATOR_H_
#define SUCCESSOR_GENERATOR_H_

#include <random>
#include <utility>
#include <vector>

#include "problem.h"

namespace pplanner {

class SuccessorGenerator {
 public:
  SuccessorGenerator() : translator_(nullptr), engine_(nullptr) {}

  explicit SuccessorGenerator(const Problem &problem)
    : fact_translator_(nullptr), engine_(nullptr) { Init(problem); }

  void Init(const Problem &problem);

  void Generate(const std::vector<int> &state, std::vector<int> &result) const;

  int Sample(const std::vector<int> &state) const;

  void Print() const;

 private:
  void Insert(const Problem &problem, int query,
              std::vector<std::pair<int, int> > &precondition);

  void AddQuery(int index, int query);

  void DFS(const std::vector<int> &state, int index, size_t current,
           std::vector<int> &result);

  void DFSample(const std::vector<int> &state, int index,  size_t current,
                unsigned int *k, int *result);

  std::vector<int> to_child_;
  std::vector<int> to_data_;
  std::vector<std::vector<int> > data_;
  std::shared_ptr<const FactTranslator> fact_translator_;
  std::unique_ptr<std::mt19937> engine_;
};

} // namespace pplanner

#endif // SUCCESSOR_GENERATOR_H_
