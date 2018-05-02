#ifndef RPG_TABLE_H_
#define RPG_TABLE_H_

#include <queue>
#include <utility>
#include <vector>

#include "sas_plus/relaxed_sas_plus.h"

namespace pplanner {

class RPGTable {
 public:
  RPGTable() : problem_(nullptr) {}

  RPGTable(std::shared_ptr<const RelaxedSASPlus> problem)
    : goal_counter_(problem.n_goal_facts()),
      op_cost_(problem.n_actions(), -1),
      precondition_counter_(problem.n_actions(), -1),
      prop_cost_(problem.n_facts(), -1),
      best_support_(problem.n_facts(), -1),
      marked_(problem.n_actions(), false),
      plan_set_(problem.n_actions(), false),
      problem_(problem) {}

  int PlanCost(const std::vector<int> &state);

  int PlanCost(const std::vector<int> &state,
               std::unordered_set<int> &preferred);

  int AdditiveCost(const std::vector<int> &state);

 private:
  void SetPlan(int g);

  void GeneralizedDijkstra(const std::vector<int> &state);

  void SetUp(const std::vector<int> &state);

  void MayPush(int f, int a);

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
  PQueue q_;
  std::shared_ptr<const RelaxedSASPlus> problem_;
};

} // namespace pplanner

#endif // RPG_TABLE_H_
