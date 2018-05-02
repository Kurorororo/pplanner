#ifndef BLIND_H_
#define BLIND_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "sas_plus.h"
#include "search_graph.h"

namespace pplanner {

class Blind : public Evaluator {
 public:
  Blind() : problem_(nullptr), graph_(nullptr) {}

  Blind(std::shared_ptr<const SASPlus> problem,
        std::shared_ptr<const SearchGraph> graph)
      : problem_(problem), graph_(graph) { Init(); }

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

 private:
  void Init();

  int cheapest_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<const SearchGraph> graph_;
};

} // namespace pplanner

#endif // BLIND_H_
