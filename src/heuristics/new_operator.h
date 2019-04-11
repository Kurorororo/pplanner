#ifndef NEW_OPERATOR_H_
#define NEW_OPERATOR_H_

#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "sas_plus.h"
#include "search_graph.h"
#include "search_node.h"

namespace pplanner {

class NewOperator : public Evaluator {
 public:
  NewOperator() {}

  NewOperator(std::shared_ptr<const SASPlus> problem)
      : is_new_(problem->n_actions(), true) {}

  ~NewOperator() {}

  int Evaluate(const std::vector<int> &state, int node) override { return 1; }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override;

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
  int Evaluate(const std::vector<int> &state,
               std::shared_ptr<SearchNode> node) override {
    return Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state, std::shared_ptr<SearchNode> node,
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

  std::vector<bool> is_new_;
};

}  // namespace pplanner

#endif  // NEW_OPERATOR_H_
