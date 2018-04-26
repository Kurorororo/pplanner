#include "sas_plus.h"

#include "sas_plus/parse_utils.h"

using std::vector;
using std::pair;

namespace pplanner {

void SASPlus::CreateActions(int n) {
  action_costs_.reserve(n);
  action_names_.reserve(n);
  preconditions_ = std::make_shared(new PartialStateVector);
  effects_ = std::make_shared(new EffectVector);
  preconditions_->Rserve(n);
  effects->Rserve(n);
}

int SASPlus::AddAction(int cost, const string &name,
                       vector<pair<int, int> > &precondition,
                       vector<pair<int, int> > &effect) {
  int a = static_cast<int>(action_costs_.size());

  action_costs_.push_back(cost);
  action_names_.push_back(name);
  preconditions_->Add(precondition);
  effects_->Add(effect);

  return a;
}

void SASPlus::InitFromLines(queue<string> &lines) {
  int version = ParseVersion(lines);

  metric_ = ParseMetric(lines);

  int n = ParseN(lines);
  facts_ = std::make_shared(new Facts);
  facts_->Reserve(n);

  for (int i=0; i<n; ++i)
    facts_->Add(ParseVariables(lines));

  n = ParseN(lines);
  mutex_groups_ = std::make_shared(new MutexGroups);
  mutex_groups_->Reserve(n);

  for (int i=0; i<n; ++i)
    mutex_groups_.Add(ParseMutexGroup(*facts_, lines));

  initial_ = ParseInitial(*facts_, lines);

  goal_ = std::make_shared(ParseGoal(lines));

  n = ParseN(lines);
  CreateActions(n);

  string name;
  vector<pair<int, int> > precondition;
  vector<pair<int, int> > effect;

  for (int i=0; i<n; ++i) {
    int cost = ParseOperator(lines, name, precondition, effect);
    if (metric_ == 0) cost = 1;
    AddAction(cost, name, precondition, effect);
  }

  n = ParseN(lines);

  for (int i=0; i<n; ++i)
    ParseAxiom(lines);
}

void SASPlus::CopyPrecondition(int i, vector<pair<int, int> > &precondition)
  const {
  size_t size = preconditions_->SizeOfPartialState(i);
  partial_state.resize(size);

  auto var_iter = preconditions_->VarsBegin(i);
  auto value_iter = preconditions_->ValuesBegin(i);

  for (auto &p : partial_state) {
    p.first = *var_iter;
    p.second *value_iter;
    ++var_iter;
    ++value_iter;
  }
}

} // namespace pplanner
