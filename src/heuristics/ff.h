#ifndef FF_H_
#define FF_H_

#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "heuristics/hn_rpg.h"
#include "heuristics/relaxed_sas_plus.h"
#include "random_walk_evaluator.h"
#include "sas_plus.h"
#include "search_graph.h"
#include "search_node.h"

namespace pplanner {

class FF : public Evaluator {
 public:
  FF() : problem_(nullptr), r_problem_(nullptr), rpg_(nullptr) {}

  FF(std::shared_ptr<const SASPlus> problem, bool simplify = false,
     bool unit_cost = false)
      : problem_(problem),
        r_problem_(
            std::make_shared<RelaxedSASPlus>(problem, simplify, unit_cost)),
        rpg_(nullptr) {
    rpg_ = std::unique_ptr<HNRPG>(new HNRPG(r_problem_));
  }

  ~FF() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    StateToFactVector(problem_, state, facts_);

    return rpg_->PlanCost(facts_);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    StateToFactVector(problem_, state, facts_);

    return rpg_->PlanCost(facts_, preferred);
  }

  // for MPI
  int Evaluate(const std::vector<int> &state, int node, int parent) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node, applicable, preferred);
  }

  // for multithread
  int Evaluate(const std::vector<int> &state, SearchNode *node) override {
    return Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state, SearchNode *node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, -1, applicable, preferred);
  }

  // for random walk
  int Evaluate(const std::vector<int> &state) override {
    return Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, -1, applicable, preferred);
  }

  void UpdateBest() override {}

  void LocalRestart() override {}

  void GlobalRestart() override {}

  void CopyBestToSearchGraph(int node,
                             std::shared_ptr<SearchGraph> graph) override {}

 private:
  std::vector<int> facts_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::unique_ptr<HNRPG> rpg_;
};

}  // namespace pplanner

#endif  // FF_H_
