#ifndef BLIND_H_
#define BLIND_H_

#include <memory>
#include <vector>

#include "evaluator.h"

namespace pplanner {

class Blind : public Evaluator {
 public:
  Blind() : problem_(nullptr), graph_(nullptr) {}

  Blind(std::shared_ptr<const SASPlus> problem,
        std::shared_ptr<const SearchGraph> graph)
      : problem_(problem), graph_(graph) { Init(); }

  ~Blind() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    if (probelm_->IsGoal(state)) return 0;

    return cheapest_;
  }

 private:
  void Init();

  int cheapest_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<const SearchGraph> graph_;
};

} // namespace pplanner

#endif // BLIND_H_
