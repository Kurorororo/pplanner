#ifndef BLIND_H_
#define BLIND_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "sas_plus.h"
#include "search_graph.h"
#include "search_node.h"

namespace pplanner {

class Blind : public Evaluator {
 public:
  Blind() : problem_(nullptr) {}

  Blind(std::shared_ptr<const SASPlus> problem) : problem_(problem) { Init(); }

  ~Blind() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    if (problem_->IsGoal(state)) return 0;

    return cheapest_;
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node);
  }

  // for MPI
  int Evaluate(const std::vector<int> &state, int node, int parent) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node);
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

 private:
  void Init();

  int cheapest_;
  std::shared_ptr<const SASPlus> problem_;
};

}  // namespace pplanner

#endif  // BLIND_H_
