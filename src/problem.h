#ifndef PROBLEM_H_
#define PROBLEM_H_

#include <string>
#include <unordered_set>
#include <vector>

#include "problem/effect_vector.h"
#include "problem/facts.h"
#include "problem/mutex_groups.h"
#include "problem/partial_state.h"
#include "problem/partial_state_vector.h"

namespace pplanner {

class Problem {
 public:
  Problem() : metric_(0),
              goal_(nullptr),
              preconditions_(nullptr),
              effects_(nullptr),
              successor_generator_(nullptr),
              translator_(nullptr),
              mutex_groups_(nullptr) {}

  void CreateGoal(size_t size) {
    goal_ = std::make_shared(new PartialState(size));
  }

  void CreateActions(size_t size) {
    preconditions_ = std::make_shared(new PartialStateVector(size));
    effects_ = std::make_shared(new EffectVector(size));
  }

  void CreateSuccssorGenerator() {
    assert(preconditions_ != nullptr);

    successor_generator_ = std::make_shared(
        new SuccessorGenerator(*preconditions_);
  }

  void CreateFactTranslator() {
    assert(!initial_.empty());

    translator_ = std::make_shared(new FactTranslator(n_variables()));
  }

  void CreateMutexGroups(size_t size) {
    mutex_groups_ = std::make_shared(new MutexGroups(size));
  }

  size_t n_variables() const { return initial_.size(); }

  size_t n_goal_facts() const { return partial_state->size(); }

  size_t n_actions() const { return costs_.size(); }

  size_t n_facts() const { return translator_->n_facts(); }

  int metric() const { return metric_; }

  void set_metric(int metric) { metric_ = metric };

  const std::vector<int>& initial() const { return initial_; }

  void set_initial(const std::vector<int> &state) { initial_ = state; }

  int ActionCost(int i) const { return action_costs_[i]; }

  void AddActionCost(int cost) { action_costs_.push_back(cost); }

  const std::string& ActionName(int i) const { return action_names_[i]; }

  void AddActionName(const std::string &name) { action_names_.push_back(name); }

  void AddFactPredicate(const std::string &predicate) {
    fact_predicates_.push_back(predicate);
  }

  const std::string& FactPredicate(int i) const { return fact_predicates_[i]; }

  void InsertToMutexGroup(int i, int fact) { mutex_groups_->Insert(i, fact); }

  void AddGoalFact(int var, int value) { goal_->Add(var, value); }

  void AddPrecondition(const std::vector<int> &vars,
                       const std::vector<int> &values) {
    preconditions_->Add(vars, values_);
  }

  void AddEffect(const std::vector<int> &vars,
                 const std::vector<int> &values) {
    effects_->Add(vars, values_);
  }

  void AddFacts(int range) { translator_->Add(range); }

  void FindApplicableActions(const std::vector<int> &state,
                             std::vector<int> &actions) const {
    successor_generator_->Find(state, actions);
  }

  void ApplyEffect(int i, std::vector<int> &state) const {
    effects_->Apply(i, state);
  }

  bool IsGoal(const std::vector<int> &state) const {
    return goal_->IsSubset(state);
  }

  bool IsMutex(int f, int g) const { return mutex_groups_->IsMutex(f, g) };

  const int* action_costs_data() { return action_costs_.data(); }

  std::shared_ptr<const PartialState> goal() { return goal_; }

  std::shared_ptr<const PartialStateVector> preconditions() {
    return preconditions_;
  }

  std::shared_ptr<const EffectVector> effects() { return effects_; }

  std::shared_ptr<const SuccessorGenerator> successor_generator() {
    return successor_generator_;
  }

  std::shared_ptr<const FactTranslator> translator() { return translator_; }

 private:
  int metric_;
  std::vector<int> initial_;
  std::vector<int> action_costs_;
  std::vector<std::string> action_names_;
  std::vector<std::string> fact_predicates_;
  std::shared_ptr<PartialState> goal_;
  std::shared_ptr<PartialStateVector> preconditions_;
  std::shared_ptr<EffectVector> effects_;
  std::shared_ptr<SuccessorGenerator> successor_generator_;
  std::shared_ptr<FactTranslator> translator_;
  std::shared_ptr<MutexGroups> mutex_groups_;
};

} // namespace pplanner

#endif // PROBLEM_H_
