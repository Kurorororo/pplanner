#ifndef M_HMAX_H_
#define M_HMAX_H_

#include <unordered_set>
#include <vector>

#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg_table.h"
#include "multithread_search/heuristic.h"
#include "multithread_search/search_node.h"
#include "sas_plus.h"

namespace pplanner {

template<typename T>
class MHmax : public Heuristic<T> {
 public:
  MHmax() : unit_cost_(false), problem_(nullptr), r_problem_(nullptr),
            rpg_(nullptr) {}

  MHmax(std::shared_ptr<const SASPlus> problem, bool simplify=true,
        bool unit_cost=false)
    : unit_cost_(unit_cost),
      problem_(problem),
      r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
      rpg_(nullptr) {
    rpg_ = std::unique_ptr<RPGTable>(new RPGTable(problem, r_problem_));
  }

  ~MHmax() {}

  int Evaluate(const std::vector<int> &state, T node) override {
    StateToFactVector(*problem_, state, facts_);

    return rpg_->HmaxCost(facts_, unit_cost_);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred, T node) override {
    return Evaluate(state, node);
  }

 private:
  bool unit_cost_;
  std::vector<int> facts_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::unique_ptr<RPGTable> rpg_;
};

} // namespace pplanner

#endif // M_HMAX_H_
