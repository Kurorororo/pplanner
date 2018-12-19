#include "sas_plus.h"

#include <algorithm>
#include <iostream>

#include "sas_plus/parse_utils.h"

using std::make_shared;
using std::pair;
using std::queue;
using std::string;
using std::vector;

namespace pplanner {

void SASPlus::CreateActions(int n) {
  action_costs_.reserve(n);
  action_names_.reserve(n);
  preconditions_ = make_shared<PartialStateVector>();
  effects_ = make_shared<EffectVector>();
  preconditions_->Reserve(n);
  effects_->Reserve(n);
}

int SASPlus::AddAction(
    int cost,
    const string &name,
    const vector<pair<int, int> > &precondition,
    const vector<pair<int, int> > &effect,
    const vector<vector<pair<int, int> > > &effect_conditions,
    const vector<pair<int, int> > &conditional_effects) {
  int a = static_cast<int>(action_costs_.size());

  action_costs_.push_back(cost);
  action_names_.push_back(name);
  auto sorted_precondition = precondition;
  std::sort(sorted_precondition.begin(), sorted_precondition.end());
  preconditions_->Add(sorted_precondition);
  auto sorted_effect = effect;
  std::sort(sorted_effect.begin(), sorted_effect.end());
  effects_->Add(sorted_effect);
  effects_->AddConditionalEffect(effect_conditions, conditional_effects);

  return a;
}

void SASPlus::InitFromLines(queue<string> &lines) {
  ParseVersion(lines);

  metric_ = ParseMetric(lines);

  int n = ParseN(lines);
  facts_ = make_shared<Facts>();
  facts_->Reserve(n);

  for (int i=0; i<n; ++i)
    facts_->AddVariable(ParseVariable(lines));

  n = ParseN(lines);
  mutex_groups_ = make_shared<MutexGroups>();
  mutex_groups_->Reserve(n);

  for (int i=0; i<n; ++i)
    mutex_groups_->AddGroup(ParseMutexGroup(*facts_, lines));

  initial_ = ParseInitial(*facts_, lines);

  auto goal = ParseGoal(lines);
  goal_ = make_shared<PartialState>(goal);

  n = ParseN(lines);
  CreateActions(n);

  string name;
  vector<pair<int, int> > precondition;
  vector<pair<int, int> > effect;
  vector<vector<pair<int, int> > > effect_conditions;
  vector<pair<int, int> > conditional_effects;

  for (int i=0; i<n; ++i) {
    int cost = ParseOperator(lines, metric_, name, precondition, effect,
                             effect_conditions, conditional_effects);
    AddAction(cost, name, precondition, effect, effect_conditions,
              conditional_effects);
  }

  n = ParseN(lines);

  for (int i=0; i<n; ++i)
    ParseAxiom(lines);
}

void SASPlus::Dump() const {
  assert(facts_ != nullptr);
  assert(mutex_groups_ != nullptr);
  assert(goal_ != nullptr);
  assert(preconditions_ != nullptr);
  assert(effects_ != nullptr);

  facts_->Dump();

  std::cout << std::endl;
  std::cout << "Intiial state" << std::endl;
  auto initial_state = initial();

  for (int i=0, n=initial_state.size(); i<n; ++i)
    std::cout << "var" << i << "=" << initial_state[i] << std::endl;

  std::cout << std::endl;
  mutex_groups_->Dump();

  std::cout << std::endl;
  std::cout << "Goal" << std::endl;
  goal_->Dump();

  std::cout << std::endl;
  int n = static_cast<int>(n_actions());
  std::cout << n << " operators" << std::endl;
  std::cout << std::endl;

  for (int i=0; i<n; ++i) {
    std::cout << "id=" << i << std::endl;
    std::cout << ActionName(i) << std::endl;
    std::cout << "cost = " << ActionCost(i) << std::endl;
    std::cout << "precondition" << std::endl;
    preconditions_->Dump(i);
    std::cout << "effect" << std::endl;
    effects_->Dump(i);
    std::cout << std::endl;
  }
}

void StateToFactVector(const SASPlus &problem, const vector<int> &state,
                       vector<int> &v) {
  v.clear();

  int i = 0;

  for (auto value : state) {
    int f = problem.Fact(i, value);
    v.push_back(f);
    ++i;
  }
}

void StateToFactSet(const SASPlus &problem, const vector<int> &state,
                    vector<bool> &s) {
  s.clear();
  s.resize(problem.n_facts(), false);

  int i = 0;

  for (auto value : state) {
    int f = problem.Fact(i, value);
    s[f] = true;
    ++i;
  }
}

} // namespace pplanner
