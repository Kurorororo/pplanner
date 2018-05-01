#ifndef EVALUATOR_H_
#define EVALUATOR_H_

namespace pplanner {

class Evaluator {
 public:
  virtual ~Evaluator() = 0;

  virtual int Evaluate(const std::vector<int> &state, int node) = 0;
};

} // namespace pplanner

#endif // EVALUATOR_H_
