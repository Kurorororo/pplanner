#ifndef RPG_H_
#define RPG_H_

#include <array>
#include <memory>
#include <unordered_set>
#include <vector>

#include "relaxed_sas_plus.h"

namespace pplanner {

class RPG {
 public:
  RPG() : problem_(nullptr) {}

  explicit RPG(std::shared_ptr<const RelaxedSASPlus> problem)
    : n_layers_(0),
      goal_counter_(problem->n_goal_facts()),
      fact_layer_membership_(problem->n_facts(), -1),
      action_layer_membership_(problem->n_actions(), 0),
      precondition_counter_(problem->n_actions(), -1),
      closed_(problem->n_facts(), false),
      selected_(problem->n_facts(), false),
      is_in_(problem->n_actions(), false),
      problem_(problem) {
        marked_[0] = std::vector<bool>(problem->n_facts(), false);
        marked_[1] = std::vector<bool>(problem->n_facts(), false);
    }

  std::vector<int> Plan(const std::vector<int> &state,
                        std::unordered_set<int> &helpful,
                        bool common_precond=false);

  int PlanCost(const std::vector<int> &state, bool unit_cost=false,
               bool common_precond=false);

  int PlanCost(const std::vector<int> &state, std::unordered_set<int> &helpful,
               bool unit_cost=false, bool common_precond=false);

  void ConstructRRPG(const std::vector<int> &state,
                     const std::vector<bool> &black_list);

  bool IsInFact(int fact) const { return fact_layer_membership_[fact] != -1; }

  bool IsInAction(int action) const { return is_in_[action]; }
private:
  void ConstructGraph(const std::vector<int> &state);

  void Reset();

  bool FactLayer();

  void ActionLayer();

  void RistrictedFactLayer(const std::vector<bool> &black_list);

  void InitializeGSet();

  std::vector<int> ExtractPlan(bool common_precond);

  int ExtractCost(bool unit_cost, bool common_precond);

  int ExtractAction(int i, int g, bool common_precond);

  int ChooseAction(int index, int i, bool common_precond) const;

  void ExtractHelpful(std::unordered_set<int> &helpful);

  int n_layers_;
  int goal_counter_;
  std::vector<int> fact_layer_membership_;
  std::vector<int> action_layer_membership_;
  std::vector<int> precondition_counter_;
  std::vector<bool> closed_;
  std::vector<int> scheduled_facts_;
  std::vector<int> scheduled_actions_;
  std::vector< std::vector<int> > g_set_;
  std::array<std::vector<bool>, 2> marked_;
  std::vector<bool> selected_;
  std::vector<bool> is_in_;
  std::shared_ptr<const RelaxedSASPlus> problem_;
};

} // namespace pplanner

#endif // RPG_H_
