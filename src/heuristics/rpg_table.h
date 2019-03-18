#ifndef RPG_TABLE_H_
#define RPG_TABLE_H_

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg.h"
#include "open_lists/single_open_list.h"
#include "utils/priority_queue.h"

namespace pplanner {

class RPGTable : public RPG {
 public:
  RPGTable()
      : unit_cost_(false),
        more_helpful_(false),
        q_(nullptr),
        problem_(nullptr),
        r_problem_(nullptr) {}

  RPGTable(std::shared_ptr<const SASPlus> problem,
           std::shared_ptr<const RelaxedSASPlus> r_problem,
           std::string tie_break = "cpp", bool more_helpful = false)
      : unit_cost_(r_problem->unit_cost()),
        more_helpful_(more_helpful),
        goal_counter_(r_problem->n_goal_facts()),
        op_cost_(r_problem->n_actions(), -1),
        precondition_counter_(r_problem->n_actions(), -1),
        prop_cost_(r_problem->n_facts(), -1),
        best_support_(r_problem->n_facts(), -1),
        marked_(r_problem->n_actions(), false),
        plan_set_(problem->n_actions(), false),
        is_applicable_(problem->n_actions(), false),
        is_disjunctive_goal_(problem->n_facts(), false),
        supporters_(problem->n_facts()),
        q_(nullptr),
        problem_(problem),
        r_problem_(r_problem) {
    if (r_problem->unit_cost())
      q_ = VectorPriorityQueueFactory<int>(tie_break);
    else
      q_ = PriorityQueueFactory<int, int>(tie_break);
  }

  ~RPGTable() {}

  int PlanCost(const std::vector<int> &state) override {
    return PlanCost(state, helpful_);
  }

  int PlanCost(const std::vector<int> &state,
               std::unordered_set<int> &helpful) override;

  void ConstructRRPG(const std::vector<int> &state,
                     const std::vector<bool> &black_list) override;

  bool IsIn(int fact) const override { return prop_cost_[fact] != -1; }

  bool IsApplicable(int action) const override {
    return is_applicable_[action];
  }

  void DisjunctiveHelpful(const std::vector<int> &state,
                          const std::vector<int> &disjunctive_goals,
                          std::unordered_set<int> &helpful) override;

  int AdditiveCost(const std::vector<int> &state);

  int HmaxCost(const std::vector<int> &state);

 private:
  void SetPlan(int g, std::unordered_set<int> &helpful);

  void GeneralizedDijkstra(const std::vector<int> &state, bool hmax = false);

  void SetUp(const std::vector<int> &state);

  void MayPush(int f, int a);

  int DisjunctiveGeneralizedDijkstra(const std::vector<int> &state);

  bool unit_cost_;
  bool more_helpful_;
  int goal_counter_;
  std::vector<int> op_cost_;
  std::vector<int> precondition_counter_;
  std::vector<int> prop_cost_;
  std::vector<int> best_support_;
  std::vector<bool> marked_;
  std::vector<bool> plan_set_;
  std::vector<bool> is_applicable_;
  std::vector<bool> is_disjunctive_goal_;
  std::vector<std::vector<int> > supporters_;
  std::unordered_set<int> helpful_;
  std::unique_ptr<PriorityQueue<int, int> > q_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<const RelaxedSASPlus> r_problem_;
  std::vector<int> values_;
};

}  // namespace pplanner

#endif  // RPG_TABLE_H_
