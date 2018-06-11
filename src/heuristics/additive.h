#ifndef ADDITIVE_H_
#define ADDITIVE_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "random_walk_evaluator.h"
#include "sas_plus.h"
#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg_table.h"

namespace pplanner {

class Additive : public Evaluator {
 public:
  Additive() : unit_cost_(false), problem_(nullptr), r_problem_(nullptr),
               rpg_(nullptr) {}

  Additive(std::shared_ptr<const SASPlus> problem, bool simplify=true,
           bool unit_cost=false)
    : unit_cost_(unit_cost),
      problem_(problem),
      r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
      rpg_(nullptr) {
    rpg_ = std::unique_ptr<RPGTable>(new RPGTable(problem, r_problem_));
  }

  ~Additive() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->AdditiveCost(facts_, unit_cost_);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->AdditiveCost(facts_, unit_cost_);
  }

 private:
  bool unit_cost_;
  std::vector<int> facts_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::unique_ptr<RPGTable> rpg_;
};

class RWAdditive : public RandomWalkEvaluator {
 public:
  RWAdditive() : additive_(nullptr) {
    additive_ = std::unique_ptr<Additive>(new Additive());
  }

  RWAdditive(std::shared_ptr<const SASPlus> problem, bool simplify=true,
             bool unit_cost=false) : additive_(nullptr) {
    additive_ = std::unique_ptr<Additive>(
        new Additive(problem, simplify, unit_cost));
  }

  ~RWAdditive() {}

  int Evaluate(const std::vector<int> &state) override {
    return additive_->Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return additive_->Evaluate(state, -1, applicable, preferred);
  }

  void UpdateBest() override {}

  void LocalRestart() override {}

  void GlobalRestart() override {}

  private:
   std::unique_ptr<Additive> additive_;
};


} // namespace pplanner

#endif // ADDITIVE_H_
