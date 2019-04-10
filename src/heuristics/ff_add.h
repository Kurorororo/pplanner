#ifndef FF_ADD_H_
#define FF_ADD_H_

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg_table.h"
#include "sas_plus.h"
#include "search_graph.h"
#include "search_node.h"

namespace pplanner {

class FFAdd : public Evaluator {
 public:
  FFAdd() : problem_(nullptr), r_problem_(nullptr), rpg_(nullptr) {}

  FFAdd(std::shared_ptr<const SASPlus> problem, bool simplify = true,
        bool unit_cost = false, std::string tie_break = "cpp",
        bool more_helpful = false)
      : problem_(problem),
        r_problem_(
            std::make_shared<RelaxedSASPlus>(problem, simplify, unit_cost)),
        rpg_(std::make_unique<RPGTable>(problem, r_problem_, tie_break,
                                        more_helpful)) {}

  ~FFAdd() {}

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
  std::unique_ptr<RPGTable> rpg_;
};

}  // namespace pplanner

#endif  // FF_ADD_H_
