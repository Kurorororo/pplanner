#ifndef BLIND_H_
#define BLIND_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "random_walk_evaluator.h"
#include "sas_plus.h"

namespace pplanner {

class Blind : public Evaluator {
 public:
  Blind() : problem_(nullptr) {}

  Blind(std::shared_ptr<const SASPlus> problem)
      : problem_(problem) { Init(); }

  ~Blind() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    if (problem_->IsGoal(state)) return 0;

    return cheapest_;
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
    return Evaluate(state, node);
  }

 private:
  void Init();

  int cheapest_;
  std::shared_ptr<const SASPlus> problem_;
};

class RWBlind : public RandomWalkEvaluator {
 public:
  RWBlind() : blind_(nullptr) {
    blind_ = std::unique_ptr<Blind>(new Blind());
  }

  RWBlind(std::shared_ptr<const SASPlus> problem) : blind_(nullptr) {
    blind_ = std::unique_ptr<Blind>(new Blind(problem));
  }

  ~RWBlind() {}

  int Evaluate(const std::vector<int> &state) override {
    return blind_->Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return blind_->Evaluate(state, -1, applicable, preferred);
  }

  void UpdateBest() override {}

  void LocalRestart() override {}

  void GlobalRestart() override {}

  void CopyBestToSearchGraph(int node,
                             std::shared_ptr<SearchGraph> graph) override {}

 private:
  std::unique_ptr<Blind> blind_;
};

} // namespace pplanner

#endif // BLIND_H_
