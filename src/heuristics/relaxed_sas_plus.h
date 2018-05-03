#ifndef RELAXED_SAS_PLUS_H_
#define RELAXED_SAS_PLUS_H_

#include <vector>

#include "sas_plus.h"

namespace pplanner {

class RelaxedSASPlus {
 public:
  RelaxedSASPlus() {}

  RelaxedSASPlus(const SASPlus &problem, bool simplify=true) {
    Init(problem, simplify);
  }

  size_t n_facts() const { return is_goal_.size(); }

  size_t n_actions() const { return ids_.size(); }

  int n_goal_facts() const { return goal_.size(); }

  int ActionId(int i) const { return ids_[i]; }

  int ActionCost(int i) const { return costs_[i]; }

  int PreconditionSize(int i) const { return precondition_size_[i]; }

  const std::vector<int>& Precondition(int i) const { return preconditions_[i]; }

  int Effect(int i) const { return effects_[i]; }

  bool IsGoal(int i) const { return is_goal_[i]; }

  const std::vector<int>& goal() const { return goal_; }

  const std::vector<int>& PreconditionMap(int i) const {
    return precondition_map_[i];
  }

  const std::vector<int>& EffectMap(int i) const { return effect_map_[i]; }

 private:
  void Init(const SASPlus &problem, bool simplify) {
    InitActions(problem, simplify);
    InitGoal(problem);
  }

  void InitActions(const SASPlus &problem, bool simplify);

  void InitGoal(const SASPlus &problem);

  void Simplify();

  std::vector<int> ids_;
  std::vector<int> costs_;
  std::vector<int> precondition_size_;
  std::vector<std::vector<int> > preconditions_;
  std::vector<int> effects_;
  std::vector<std::vector<int> > precondition_map_;
  std::vector<std::vector<int> > effect_map_;
  std::vector<bool> is_goal_;
  std::vector<int> goal_;
};

} // namespace pplanner

#endif // RELAXED_SAS_PLUS_H_
