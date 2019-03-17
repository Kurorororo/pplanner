#include "heuristics/relaxed_sas_plus.h"

#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <utility>

#include <boost/functional/hash.hpp>

using std::pair;
using std::unordered_map;
using std::vector;

namespace pplanner {

void RelaxedSASPlus::InitActions(const SASPlus &problem, bool simplify) {
  vector<pair<int, int> > pair_precondition;
  vector<pair<int, int> > pair_effect;
  vector<int> precondition;
  vector<int> actions;
  vector<vector<pair<int, int> > > effect_conditions;
  vector<pair<int, int> > conditional_effects;
  int action = 0;

  for (int i = 0, n = problem.n_actions(); i < n; ++i) {
    problem.CopyPrecondition(i, pair_precondition);
    precondition.clear();

    for (auto &v : pair_precondition) {
      int f = problem.Fact(v.first, v.second);
      precondition.push_back(f);
    }

    int cost = problem.ActionCost(i);
    int size = precondition.size();

    problem.CopyEffect(i, pair_effect);
    actions.clear();

    for (auto &v : pair_effect) {
      int f = problem.Fact(v.first, v.second);
      ids_.push_back(i);
      actions.push_back(action++);
      costs_.push_back(cost);
      precondition_size_.push_back(size);
      preconditions_.push_back(precondition);
      effects_.push_back(f);
      conditional_.push_back(false);
    }

    if (problem.HasConditionalEffects(i)) {
      problem.CopyEffectConditions(i, effect_conditions);
      problem.CopyConditionalEffects(i, conditional_effects);

      for (int j = 0, m = effect_conditions.size(); j < m; ++j) {
        auto precondition_ec = precondition;

        for (auto &p : effect_conditions[j])
          precondition_ec.push_back(problem.Fact(p.first, p.second));

        ids_.push_back(i);
        actions.push_back(action++);
        costs_.push_back(cost);
        precondition_size_.push_back(precondition_ec.size());
        preconditions_.push_back(precondition_ec);
        auto effect = conditional_effects[j];
        effects_.push_back(problem.Fact(effect.first, effect.second));
        conditional_.push_back(true);
      }
    }

    id_to_actions_.push_back(actions);
  }

  std::cout << ids_.size() << " unary operators" << std::endl;

  if (simplify) {
    Simplify();
    std::cout << "simplified to " << ids_.size() << std::endl;
  }

  precondition_map_.resize(problem.n_facts());

  for (int i = 0, n = preconditions_.size(); i < n; ++i) {
    if (preconditions_[i].empty()) no_preconditions_.push_back(i);
    for (auto f : preconditions_[i]) precondition_map_[f].push_back(i);
  }

  effect_map_.resize(problem.n_facts());

  for (int i = 0, n = effects_.size(); i < n; ++i) {
    int f = effects_[i];
    effect_map_[f].push_back(i);
  }
}  // namespace pplanner

void RelaxedSASPlus::InitGoal(const SASPlus &problem) {
  vector<pair<int, int> > goal;
  problem.CopyGoal(goal);
  is_goal_.resize(problem.n_facts(), false);

  for (auto &v : goal) {
    int f = problem.Fact(v.first, v.second);
    is_goal_[f] = true;
    goal_.push_back(f);
  }
}

void RelaxedSASPlus::Simplify() {
  auto key_hash = [](const pair<vector<int>, int> &v) {
    auto hash = boost::hash_range(v.first.begin(), v.first.end());
    boost::hash_combine(hash, std::hash<int>()(v.second));

    return hash;
  };

  unordered_map<pair<vector<int>, int>, int, decltype(key_hash)> umap(
      n_actions(), key_hash);

  for (int i = 0, n = n_actions(); i < n; ++i) {
    pair<vector<int>, int> key;
    key.first = preconditions_[i];
    std::sort(key.first.begin(), key.first.end());
    key.second = effects_[i];

    auto entry = umap.find(key);

    if (entry == umap.end()) {
      umap[key] = i;
    } else if (costs_[i] < costs_[entry->second]) {
      entry->second = i;
    }
  }

  vector<int> ids;
  vector<std::vector<int> > id_to_actions(id_to_actions_.size());
  vector<int> costs;
  vector<int> precondition_size;
  vector<vector<int> > preconditions;
  vector<int> effects;
  vector<bool> conditional;
  int action = 0;

  for (auto &v : umap) {
    auto key = v.first;
    auto p = key.first;
    int e = key.second;
    int a = v.second;
    int cost = costs_[a];

    bool match = false;

    if (p.size() <= 5) {
      int pw_size = (1 << p.size()) - 1;

      // example: p.size() == 2
      // mask: 000, 001, 010, 011, 100, 101, 110, 111
      for (int mask = 0; mask < pw_size; ++mask) {
        auto dominate_key = std::make_pair(vector<int>(), e);

        // 1 << i: 001, 010, 100
        for (int i = 0, n = p.size(); i < n; ++i)
          if (mask & (1 << i)) dominate_key.first.push_back(p[i]);

        auto entry = umap.find(dominate_key);

        if (entry != umap.end() && costs_[entry->second] <= cost) {
          match = true;
          break;
        }
      }
    }

    if (!match) {
      ids.push_back(ids_[a]);
      id_to_actions[ids_[a]].push_back(action++);
      costs.push_back(cost);
      precondition_size.push_back(p.size());
      preconditions.push_back(p);
      effects.push_back(e);
      conditional.push_back(conditional_[a]);
    }
  }

  ids_.swap(ids);
  id_to_actions_.swap(id_to_actions);
  costs_.swap(costs);
  precondition_size_.swap(precondition_size);
  preconditions_.swap(preconditions);
  effects_.swap(effects);
  conditional_.swap(conditional);
}

}  // namespace pplanner
