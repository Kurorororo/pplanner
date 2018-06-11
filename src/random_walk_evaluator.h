#ifndef RANDOM_WALK_EVALUATOR_H_
#define RANDOM_WALK_EVALUATOR_H_

#include <unordered_set>
#include <vector>

namespace pplanner {

class RandomWalkEvaluator {
 public:
  virtual ~RandomWalkEvaluator() = 0;

  virtual int Evaluate(const std::vector<int> &state) = 0;

  virtual int Evaluate(const std::vector<int> &state,
                       const std::vector<int> &applicable,
                       std::unordered_set<int> &preferred) = 0;

  virtual void UpdateBest() = 0;

  virtual void LocalRestart() = 0;

  virtual void GlobalRestart() = 0;
};

} // namespace pplanner

#endif // RANDOM_WALK_EVALUATOR_H_
