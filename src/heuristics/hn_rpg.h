#ifndef HN_RPG_H_
#define HN_RPG_H_

#include <array>
#include <memory>
#include <unordered_set>
#include <vector>

#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg.h"

namespace pplanner {

class HNRPG : public RPG {
 public:
  HNRPG() : problem_(nullptr) {}

  explicit HNRPG(std::shared_ptr<const RelaxedSASPlus> problem)
      : n_layers_(0),
        goal_counter_(problem->n_goal_facts()),
        fact_layer_membership_(problem->n_facts(), -1),
        action_layer_membership_(problem->n_actions(), 0),
        precondition_counter_(problem->n_actions(), -1),
        closed_(problem->n_facts(), false),
        is_disjunctive_goal_(problem->n_facts(), false),
        is_applicable_(problem->n_actions(), false),
        problem_(problem) {
    marked_[0] = std::vector<bool>(problem->n_facts(), false);
    marked_[1] = std::vector<bool>(problem->n_facts(), false);
  }

  ~HNRPG() {}

  int PlanCost(const std::vector<int> &state) override;

  int PlanCost(const std::vector<int> &state,
               std::unordered_set<int> &helpful) override;

  void ConstructRRPG(const std::vector<int> &state,
                     const std::vector<bool> &black_list) override;

  bool IsIn(int fact) const override {
    return fact_layer_membership_[fact] != -1;
  }

  bool IsApplicable(int action) const override {
    return is_applicable_[action];
  }

  void DisjunctiveHelpful(const std::vector<int> &state,
                          const std::vector<int> &disjunctive_goal,
                          std::unordered_set<int> &helpful) override;

  std::vector<int> Plan(const std::vector<int> &state,
                        std::unordered_set<int> &helpful);

 private:
  void ConstructGraph(const std::vector<int> &state);

  void SetUp();

  bool FactLayer();

  void ActionLayer();

  void RistrictedFactLayer(const std::vector<bool> &black_list);

  void InitializeGSet();

  std::vector<int> ExtractPlan();

  int ExtractCost();

  int ExtractAction(int i, int g);

  int ChooseAction(int index, int i) const;

  void ExtractHelpful(std::unordered_set<int> &helpful);

  int ConstructDisjunctiveRPG(const std::vector<int> &state,
                              const std::vector<int> &disjunctive_goal,
                              std::unordered_set<int> &helpful);

  int DisjunctiveFactLayer();

  int n_layers_;
  int goal_counter_;
  std::vector<int> fact_layer_membership_;
  std::vector<int> action_layer_membership_;
  std::vector<int> precondition_counter_;
  std::vector<bool> closed_;
  std::vector<bool> is_disjunctive_goal_;
  std::vector<int> scheduled_facts_;
  std::vector<int> scheduled_actions_;
  std::vector<std::vector<int> > g_set_;
  std::array<std::vector<bool>, 2> marked_;
  std::vector<bool> is_applicable_;
  std::shared_ptr<const RelaxedSASPlus> problem_;
};

}  // namespace pplanner

#endif  // HN_RPG_H_
