#ifndef RPG_TABLE_H_
#define RPG_TABLE_H_

#include <queue>
#include <utility>
#include <vector>

#include "heuristics/relaxed_sas_plus.h"

namespace pplanner {

class RPGTable {
 public:
  RPGTable() : r_problem_(nullptr) {}

  RPGTable(std::shared_ptr<const SASPlus> problem,
           std::shared_ptr<const RelaxedSASPlus> r_problem)
    : goal_counter_(r_problem->n_goal_facts()),
      op_cost_(r_problem->n_actions(), -1),
      precondition_counter_(r_problem->n_actions(), -1),
      prop_cost_(r_problem->n_facts(), -1),
      best_support_(r_problem->n_facts(), -1),
      marked_(r_problem->n_actions(), false),
      plan_set_(problem->n_actions(), false),
      supporters_(problem->n_facts()),
      problem_(problem),
      r_problem_(r_problem) {}

  int PlanCost(const std::vector<int> &state, std::unordered_set<int> &helpful,
               bool unit_cost=false, bool more_helpful=false);

  int PlanCost(const std::vector<int> &state, bool unit_cost=false) {
    return PlanCost(state, helpful_, unit_cost, false);
  }

  int AdditiveCost(const std::vector<int> &state, bool unit_cost=false,
                   bool more_helpful=false);

 private:
  void SetPlan(int g, std::unordered_set<int> &helpful, bool more_helpful);

  void GeneralizedDijkstra(const std::vector<int> &state, bool more_helpful);

  void SetUp(const std::vector<int> &state, bool unit_cost, bool more_helpful);

  void MayPush(int f, int a, bool more_helpful);

  struct FirstGreater {
    bool operator()(const std::pair<int, int> &a, const std::pair<int, int> &b) {
      return a.first > b.first;
    }
  };

  using PQueue = std::priority_queue<std::pair<int, int>,
                                      std::vector<std::pair<int, int> >,
                                      FirstGreater>;

  int goal_counter_;
  std::vector<int> op_cost_;
  std::vector<int> precondition_counter_;
  std::vector<int> prop_cost_;
  std::vector<int> best_support_;
  std::vector<bool> marked_;
  std::vector<bool> plan_set_;
  std::vector<std::vector<int> > supporters_;
  std::unordered_set<int> helpful_;
  PQueue q_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<const RelaxedSASPlus> r_problem_;
};

} // namespace pplanner

#endif // RPG_TABLE_H_
