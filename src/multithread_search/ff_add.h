#ifndef FF_ADD_H_
#define FF_ADD_H_

#include <unordered_set>
#include <vector>

#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg_table.h"
#include "multithread_search/heuristic.h"
#include "multithread_search/search_node.h"
#include "sas_plus.h"

namespace pplanner {

template <typename T>
class MFFAdd : public Heuristic<T> {
 public:
  MFFAdd()
      : unit_cost_(false),
        problem_(nullptr),
        r_problem_(nullptr),
        rpg_(nullptr) {}

  MFFAdd(std::shared_ptr<const SASPlus> problem, bool simplify = true,
         bool unit_cost = false, bool more_helpful = false)
      : unit_cost_(unit_cost),
        problem_(problem),
        r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
        rpg_(std::make_unique<RPGTable>(problem, r_problem_, "cpp",
                                        more_helpful)) {}

  ~MFFAdd() {}

  int Evaluate(const std::vector<int> &state, T node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_, unit_cost_);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred, T node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_, preferred, unit_cost_);
  }

 private:
  bool unit_cost_;
  std::vector<int> facts_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::unique_ptr<RPGTable> rpg_;
};

}  // namespace pplanner

#endif  // FF_ADD_H_
