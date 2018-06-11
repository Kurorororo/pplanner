#ifndef NEW_OPERATOR_H_
#define NEW_OPERATOR_H_

#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "random_walk_evaluator.h"
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

class RWNewOperator : public RandomWalkEvaluator {
 public:
  RWNewOperator() : new_operator_(nullptr) {
    new_operator_ = std::unique_ptr<NewOperator>(new NewOperator());
  }

  RWNewOperator(std::shared_ptr<const SASPlus> problem)
    : new_operator_(nullptr) {
    new_operator_ = std::unique_ptr<NewOperator>(new NewOperator(problem));
  }

  ~RWNewOperator() {}

  int Evaluate(const std::vector<int> &state) override {
    return new_operator_->Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return new_operator_->Evaluate(state, -1, applicable, preferred);
  }

  void UpdateBest() override {}

  void RollBackBest() override {}

  void RollBackInitial() override {}

 private:
  std::unique_ptr<NewOperator> new_operator_;
};

} // namespace pplanner

#endif // NEW_OPERATOR_H_
