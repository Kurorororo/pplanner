#ifndef M_FF_H_
#define M_FF_H_

#include <unordered_set>
#include <vector>

#include "heuristics/hn_rpg.h"
#include "heuristics/relaxed_sas_plus.h"
#include "multithread_search/heuristic.h"
#include "multithread_search/search_node.h"
#include "sas_plus.h"

namespace pplanner {

template <typename T>
class MFF : public Heuristic<T> {
 public:
  MFF() : problem_(nullptr), r_problem_(nullptr), rpg_(nullptr) {}

  MFF(std::shared_ptr<const SASPlus> problem, bool simplify = false,
      bool unit_cost = false)
      : problem_(problem),
        r_problem_(
            std::make_shared<RelaxedSASPlus>(problem, simplify, unit_cost)),
        rpg_(std::make_unique<HNRPG>(r_problem_)) {}

  ~MFF() {}

  int Evaluate(const std::vector<int> &state, T node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred, T node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_, preferred);
  }

 private:
  std::vector<int> facts_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::unique_ptr<HNRPG> rpg_;
};

}  // namespace pplanner

#endif  // M_FF_H_
