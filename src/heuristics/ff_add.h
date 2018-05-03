#ifndef FF_ADD_H_
#define FF_ADD_H_

#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "sas_plus.h"
#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg_table.h"

namespace pplanner {

class FFAdd : public Evaluator {
 public:
  FFAdd() : problem_(nullptr), r_problem_(nullptr), rpg_(nullptr) {}

  FFAdd(std::shared_ptr<const SASPlus> problem, bool simplify=true)
    : problem_(problem),
      r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
      rpg_(nullptr) {
    rpg_ = std::unique_ptr<RPGTable>(new RPGTable(r_problem_));
  }

  ~FFAdd() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->PlanCost(facts_, applicable, preferred);
  }

 private:
  std::vector<int> facts_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::unique_ptr<RPGTable> rpg_;
};

} // namespace pplanner

#endif // FF_ADD_H_
