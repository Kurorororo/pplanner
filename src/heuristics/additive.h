#ifndef ADDITIVE_H_
#define ADDITIVE_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "sas_plus.h"
#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg_table.h"

namespace pplanner {

class Additive : public Evaluator {
 public:
  Additive() : problem_(nullptr), r_problem_(nullptr), rpg_(nullptr) {}

  Additive(std::shared_ptr<const SASPlus> problem, bool simplify=true)
    : problem_(problem),
      r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
      rpg_(nullptr) {
    rpg_ = std::unique_ptr<RPGTable>(new RPGTable(problem, r_problem_));
  }

  ~Additive() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->AdditiveCost(facts_);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->AdditiveCost(facts_);
  }

 private:
  std::vector<int> facts_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::unique_ptr<RPGTable> rpg_;
};


} // namespace pplanner

#endif // ADDITIVE_H_
