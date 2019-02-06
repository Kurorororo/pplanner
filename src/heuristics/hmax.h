#ifndef HMAX_H_
#define HMAX_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "random_walk_evaluator.h"
#include "sas_plus.h"
#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg_table.h"

namespace pplanner {

class Hmax : public Evaluator {
 public:
  Hmax() : unit_cost_(false), problem_(nullptr), r_problem_(nullptr),
               rpg_(nullptr) {}

  Hmax(std::shared_ptr<const SASPlus> problem, bool simplify=true,
           bool unit_cost=false)
    : unit_cost_(unit_cost),
      problem_(problem),
      r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
      rpg_(nullptr) {
    rpg_ = std::unique_ptr<RPGTable>(new RPGTable(problem, r_problem_));
  }

  ~Hmax() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->HmaxCost(facts_, unit_cost_);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node, applicable, preferred);
  }

 private:
  bool unit_cost_;
  std::vector<int> facts_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::unique_ptr<RPGTable> rpg_;
};

class RWHmax : public RandomWalkEvaluator {
 public:
  RWHmax() : hmax_(nullptr) {
    hmax_ = std::unique_ptr<Hmax>(new Hmax());
  }

  RWHmax(std::shared_ptr<const SASPlus> problem, bool simplify=true,
             bool unit_cost=false) : hmax_(nullptr) {
    hmax_ = std::unique_ptr<Hmax>(
        new Hmax(problem, simplify, unit_cost));
  }

  ~RWHmax() {}

  int Evaluate(const std::vector<int> &state) override {
    return hmax_->Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return hmax_->Evaluate(state, -1, applicable, preferred);
  }

  void UpdateBest() override {}

  void LocalRestart() override {}

  void GlobalRestart() override {}

  void CopyBestToSearchGraph(int node,
                             std::shared_ptr<SearchGraph> graph) override {}

  private:
   std::unique_ptr<Hmax> hmax_;
};


} // namespace pplanner

#endif // HMAX_H_
