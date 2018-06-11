#ifndef FF_H_
#define FF_H_

#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "random_walk_evaluator.h"
#include "sas_plus.h"
#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg.h"

namespace pplanner {

class FF : public Evaluator {
 public:
  FF() : unit_cost_(false), common_precond_(false), problem_(nullptr),
         r_problem_(nullptr), rpg_(nullptr) {}

  FF(std::shared_ptr<const SASPlus> problem, bool simplify=false,
     bool unit_cost=false, bool common_precond=false)
    : unit_cost_(unit_cost),
      common_precond_(common_precond),
      problem_(problem),
      r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
      rpg_(nullptr) {
    rpg_ = std::unique_ptr<RPG>(new RPG(r_problem_));
  }

  ~FF() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_, unit_cost_, common_precond_);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_, preferred, unit_cost_, common_precond_);
  }

 private:
  bool unit_cost_;
  bool common_precond_;
  std::vector<int> facts_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::unique_ptr<RPG> rpg_;
};

class RWFF : public RandomWalkEvaluator {
 public:
  RWFF() : ff_(nullptr) {
    ff_ = std::unique_ptr<FF>(new FF());
  }

  RWFF(std::shared_ptr<const SASPlus> problem, bool simplify=false,
     bool unit_cost=false, bool common_precond=false)
    : ff_(nullptr) {
    ff_ = std::unique_ptr<FF>(
        new FF(problem, simplify, unit_cost, common_precond));
  }

  ~RWFF() {}

  int Evaluate(const std::vector<int> &state) override {
    return ff_->Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return ff_->Evaluate(state, -1, applicable, preferred);
  }

  void UpdateBest() override {}

  void RollBackBest() override {}

  void RollBackInitial() override {}

 private:
  std::unique_ptr<FF> ff_;
};

} // namespace pplanner

#endif // FF_H_
