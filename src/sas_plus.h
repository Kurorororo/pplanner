#ifndef SAS_PLUS_H_
#define SAS_PLUS_H_

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

  size_t n_variables() const { return initial_.size(); }

  size_t n_goal_facts() const {
    assert(goal_ != nullptr);

    return goal_->size();
  }

  size_t n_actions() const { return action_costs_.size(); }

  size_t n_facts() const {
    assert(facts_ != nullptr);

    return facts_->size();
  }

  int metric() const { return metric_; }

  const std::vector<int>& initial() const { return initial_; }

  int Fact(int var, int value) const { return facts_->Fact(var, value); }

  const std::string& Predicate(int var, int value) const {
    return facts_->Predicate(var, value);
  }

  bool IsMutex(int f, int g) const { return mutex_groups_->IsMutex(f, g); }

  bool IsGoal(const std::vector<int> &state) const {
    return goal_->IsSubset(state);
  }

  int ActionCost(int i) const { return action_costs_[i]; }

  const std::string& ActionName(int i) const { return action_names_[i]; }

  void CopyPrecondition(int i, std::vector<std::pair<int, int> > &precondition)
    const;

  void ApplyEffect(int i, std::vector<int> &state) const {
    effects_->Apply(i, state);
  }

  void Dump() const;

  std::shared_ptr<const Facts> facts() const { return facts_; }

  std::shared_ptr<const PartialState> goal() const { return goal_; }

  const int* action_costs_data() { return action_costs_.data(); }

  std::shared_ptr<const PartialStateVector> preconditions() const {
    return preconditions_;
  }

  std::shared_ptr<const EffectVector> effects() const { return effects_; }


 private:
  void CreateActions(int n);

  int AddAction(int cost, const std::string &name,
                const std::vector<std::pair<int, int> > &precondition,
                const std::vector<std::pair<int, int> > &effect);

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

} // namespace pplanner

#endif // SAS_PLUS_H_
