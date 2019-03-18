#ifndef RELAXED_SAS_PLUS_H_
#define RELAXED_SAS_PLUS_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class RelaxedSASPlus {
 public:
  RelaxedSASPlus() : unit_cost_(false) {}

  RelaxedSASPlus(std::shared_ptr<const SASPlus> problem, bool simplify = true,
                 bool unit_cost = false)
      : unit_cost_(unit_cost) {
    Init(problem, simplify, unit_cost);
  }

  int n_facts() const { return is_goal_.size(); }

  int n_actions() const { return ids_.size(); }

  int n_goal_facts() const { return goal_.size(); }

  bool unit_cost() const { return unit_cost_; }

  int ActionId(int i) const { return ids_[i]; }

  const std::vector<int>& IdToActions(int i) const { return id_to_actions_[i]; }

  int ActionCost(int i) const { return costs_[i]; }

  int PreconditionSize(int i) const { return precondition_size_[i]; }

  const std::vector<int>& Precondition(int i) const {
    return preconditions_[i];
  }

  int Effect(int i) const { return effects_[i]; }

  bool IsConditional(int i) const { return conditional_[i]; }

  bool IsGoal(int i) const { return is_goal_[i]; }

  const std::vector<int>& goal() const { return goal_; }

  const std::vector<int>& NoPreconditions() const { return no_preconditions_; }

  const std::vector<int>& PreconditionMap(int i) const {
    return precondition_map_[i];
  }

  const std::vector<int>& EffectMap(int i) const { return effect_map_[i]; }

 private:
  void Init(std::shared_ptr<const SASPlus> problem, bool simplify,
            bool unit_cost) {
    InitActions(problem, simplify, unit_cost);
    InitGoal(problem);
  }

  void InitActions(std::shared_ptr<const SASPlus> problem, bool simplify,
                   bool unit_cost);

  void InitGoal(std::shared_ptr<const SASPlus> problem);

  void Simplify();

  bool unit_cost_;
  std::vector<int> ids_;
  std::vector<std::vector<int> > id_to_actions_;
  std::vector<int> costs_;
  std::vector<int> precondition_size_;
  std::vector<std::vector<int> > preconditions_;
  std::vector<int> effects_;
  std::vector<bool> conditional_;
  std::vector<int> no_preconditions_;
  std::vector<std::vector<int> > precondition_map_;
  std::vector<std::vector<int> > effect_map_;
  std::vector<bool> is_goal_;
  std::vector<int> goal_;
};

}  // namespace pplanner

#endif  // RELAXED_SAS_PLUS_H_
