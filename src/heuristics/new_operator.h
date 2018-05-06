#ifndef NEW_OPERATOR_H_
#define NEW_OPERATOR_H_

#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "sas_plus.h"

namespace pplanner {

class NewOperator: public Evaluator {
 public:
  NewOperator() {}

  NewOperator(std::shared_ptr<const SASPlus> problem)
    : is_new_(problem->n_actions(), true) {}

  ~NewOperator() {}

  int Evaluate(const std::vector<int> &state, int node) override { return 1; }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override;

 private:
  std::vector<bool> is_new_;
};

} // namespace pplanner

#endif // NEW_OPERATOR_H_
