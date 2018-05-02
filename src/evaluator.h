#ifndef EVALUATOR_H_
#define EVALUATOR_H_

#include <unordered_set>
#include <vector>

namespace pplanner {

class Evaluator {
 public:
  virtual ~Evaluator() = 0;

  virtual int Evaluate(const std::vector<int> &state, int node) = 0;

  virtual int Evaluate(const std::vector<int> &state, int node,
                       const std::vector<int> &applicable,
                       std::unordered_set<int> &preferred) = 0;
};

} // namespace pplanner

#endif // EVALUATOR_H_
