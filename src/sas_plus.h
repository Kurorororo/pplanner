#ifndef SAS_PLUS_H_
#define SAS_PLUS_H_

#include <cassert>

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "sas_plus/effect_vector.h"
#include "sas_plus/facts.h"
#include "sas_plus/mutex_groups.h"
#include "sas_plus/partial_state.h"
#include "sas_plus/partial_state_vector.h"

namespace pplanner {

class SASPlus {
 public:
  SASPlus() : metric_(0),
              facts_(nullptr),
              mutex_groups_(nullptr),
              goal_(nullptr),
              preconditions_(nullptr),
              effects_(nullptr) {}

  void InitFromLines(std::queue<std::string> &lines);

  int n_variables() const { return initial_.size(); }

  int n_goal_facts() const {
    assert(goal_ != nullptr);

    return goal_->size();
  }

  int n_actions() const { return action_costs_.size(); }

  int n_facts() const {
    assert(facts_ != nullptr);

    return facts_->size();
  }

  int metric() const { return metric_; }

  bool use_conditional() const { return effects_->use_conditional(); }

  const std::vector<int>& initial() const { return initial_; }

  int Fact(int var, int value) const { return facts_->Fact(var, value); }

  int Fact(const std::pair<int, int> &v) const {
    return Fact(v.first, v.second);
  }

  int VarBegin(int var) const { return facts_->VarBegin(var); }

  int VarRange(int var) const { return facts_->VarRange(var); }

  const std::string& Predicate(int var, int value) const {
    return facts_->Predicate(var, value);
  }

  bool IsMutex(int f, int g) const { return mutex_groups_->IsMutex(f, g); }

  bool IsMutex(int l_var, int l_value, int r_var, int r_value) const {
    int f = Fact(l_var, l_value);
    int g = Fact(r_var, r_value);

    return IsMutex(f, g);
  }


  void CopyGoal(std::vector<std::pair<int, int> > &goal) const {
    goal_->Copy(goal);
  }

  bool IsGoal(const std::vector<int> &state) const {
    return goal_->IsSubset(state);
  }

  int GoalVar(int i) const { return goal_->Var(i); }

  int GoalValue(int i) const { return goal_->Value(i); }

  int ActionCost(int i) const { return action_costs_[i]; }

  const std::string& ActionName(int i) const { return action_names_[i]; }

  std::vector<int>::const_iterator PreconditionVarsBegin(int i) const {
    return preconditions_->VarsBegin(i);
  }

  std::vector<int>::const_iterator PreconditionVarsEnd(int i) const {
    return preconditions_->VarsEnd(i);
  }

  std::vector<int>::const_iterator PreconditionValuesBegin(int i) const {
    return preconditions_->ValuesBegin(i);
  }

  std::vector<int>::const_iterator PreconditionValuesEnd(int i) const {
    return preconditions_->ValuesEnd(i);
  }

  void CopyPrecondition(int i, std::vector<std::pair<int, int> > &precondition)
    const {
    preconditions_->Copy(i, precondition);
  }

  std::vector<int>::const_iterator EffectVarsBegin(int i) const {
    return effects_->VarsBegin(i);
  }

  std::vector<int>::const_iterator EffectVarsEnd(int i) const {
    return effects_->VarsEnd(i);
  }

  std::vector<int>::const_iterator EffectValuesBegin(int i) const {
    return effects_->ValuesBegin(i);
  }

  std::vector<int>::const_iterator EffectValuesEnd(int i) const {
    return effects_->ValuesEnd(i);
  }

  void CopyEffect(int i, std::vector<std::pair<int, int> > &effect) const {
    effects_->Copy(i, effect);
  }

  bool HasConditionalEffects(int i) const {
    return effects_->HasConditionalEffects(i);
  }

  int NConditionalEffects(int i) const {
    return effects_->NConditionalEffects(i);
  }

  bool EffectCondition(int i, int j, const std::vector<int> &state) const {
    return effects_->Condition(i, j, state);
  }

  int ConditionalEffectVar(int i, int j) const {
    return effects_->ConditionalEffectVar(i, j);
  }

  int ConditionalEffectValue(int i, int j) const {
    return effects_->ConditionalEffectValue(i, j);
  }

  void CopyEffectConditions(
      int i,
      std::vector<std::vector<std::pair<int, int> > > &conditions) const {
    effects_->CopyEffectConditions(i, conditions);
  }

  void CopyConditionalEffects(
      int i,
      std::vector<std::pair<int, int> > &effects) const {
    effects_->CopyConditionalEffects(i, effects);
  }

  void ApplyEffect(int i, const std::vector<int> &state,
                   std::vector<int> &child) const {
    effects_->Apply(i, state, child);
  }

  void Dump() const;

  std::shared_ptr<const Facts> facts() const { return facts_; }

  std::shared_ptr<const PartialState> goal() const { return goal_; }

  std::shared_ptr<const PartialStateVector> preconditions() const {
    return preconditions_;
  }

  std::shared_ptr<const EffectVector> effects() const { return effects_; }

  const int* var_offsets_data() const { return facts_->offsets_data(); }

  const int* goal_vars_data() const { return goal_->vars_data(); }

  const int* goal_values_data() const { return goal_->values_data(); }

  const int* action_costs_data() const { return action_costs_.data(); }

  const int* effect_offsets_data() const { return effects_->offsets_data(); }

  std::size_t effects_size() const { return effects_->n_units(); }

  const int* effect_vars_data() const { return effects_->vars_data(); }

  const int* effect_values_data() const { return effects_->values_data(); }

  const int* effect_condition_offsets_1_data() const {
    return effects_->effect_condition_offsets_1_data();
  }

  std::size_t effect_condition_offsets_2_size() const {
    return effects_->effect_condition_offsets_2_size();
  }

  const int* effect_condition_offsets_2_data() const {
    return effects_->effect_condition_offsets_2_data();
  }

  std::size_t effect_conditions_size() const {
    return effects_->effect_conditions_size();
  }

  const int* effect_condition_vars_data() const {
    return effects_->effect_condition_vars_data();
  }

  const int* effect_condition_values_data() const {
    return effects_->effect_condition_values_data();
  }

  std::size_t conditional_effects_size() const {
    return effects_->conditional_effects_size();
  }

  const int* conditional_effect_vars_data() const {
    return effects_->conditional_effect_vars_data();
  }

  const int* conditional_effect_values_data() const {
    return effects_->conditional_effect_values_data();
  }

 private:
  void CreateActions(int n);

  int AddAction(
      int cost,
      const std::string &name,
      const std::vector<std::pair<int, int> > &precondition,
      const std::vector<std::pair<int, int> > &effect,
      const std::vector<std::vector<std::pair<int, int> > > &effect_conditions,
      const std::vector<std::pair<int, int> > &conditional_effects);

  int metric_;
  std::shared_ptr<Facts> facts_;
  std::shared_ptr<MutexGroups> mutex_groups_;
  std::vector<int> initial_;
  std::shared_ptr<PartialState> goal_;
  std::vector<int> action_costs_;
  std::vector<std::string> action_names_;
  std::shared_ptr<PartialStateVector> preconditions_;
  std::shared_ptr<EffectVector> effects_;
};

void StateToFactVector(const SASPlus &problem, const std::vector<int> &state,
                       std::vector<int> &v);

void StateToFactSet(const SASPlus &problem, const std::vector<int> &state,
                    std::vector<bool> &s);

} // namespace pplanner

#endif // SAS_PLUS_H_
