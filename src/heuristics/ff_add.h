#ifndef FF_ADD_H_
#define FF_ADD_H_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg_table.h"
#include "random_walk_evaluator.h"
#include "sas_plus.h"

namespace pplanner {

class FFAdd : public Evaluator {
 public:
  FFAdd()
      : unit_cost_(false),
        problem_(nullptr),
        r_problem_(nullptr),
        rpg_(nullptr) {}

  FFAdd(std::shared_ptr<const SASPlus> problem, bool simplify = true,
        bool unit_cost = false, const std::string tie_break = "fifo",
        bool more_helpful = false)
      : unit_cost_(unit_cost),
        problem_(problem),
        r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
        rpg_(std::make_unique<RPGTable>(problem, r_problem_, tie_break,
                                        more_helpful)) {}

  ~FFAdd() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_, unit_cost_);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_, preferred, unit_cost_);
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

class RWFFAdd : public RandomWalkEvaluator {
 public:
  RWFFAdd() : ff_(nullptr) { ff_ = std::unique_ptr<FFAdd>(new FFAdd()); }

  RWFFAdd(std::shared_ptr<const SASPlus> problem, bool simplify = false,
          bool unit_cost = false, const std::string &tie_break = "cpp",
          bool more_helpful = false)
      : ff_(std::make_unique<FFAdd>(problem, simplify, unit_cost, tie_break,
                                    more_helpful)) {}

  ~RWFFAdd() {}

  int Evaluate(const std::vector<int> &state) override {
    return ff_->Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return ff_->Evaluate(state, -1, applicable, preferred);
  }

  void UpdateBest() override {}

  void LocalRestart() override {}

  void GlobalRestart() override {}

  void CopyBestToSearchGraph(int node,
                             std::shared_ptr<SearchGraph> graph) override {}

 private:
  std::unique_ptr<FFAdd> ff_;
};

}  // namespace pplanner

#endif  // FF_ADD_H_
