#include "domain/relaxed_domain.h"

#include <algorithm>
#include <iostream>
#include <utility>
#include <unordered_map>

#include <boost/functional/hash.hpp>

using std::pair;
using std::unordered_map;
using std::vector;

namespace rwls {

void Simplify(RelaxedDomain *r_domain) {
  auto key_hash = [](const pair<vector<int>, int> &v) {
    auto hash = boost::hash_range(v.first.begin(), v.first.end());
    boost::hash_combine(hash, std::hash<int>()(v.second));

    return hash;
  };

  unordered_map<pair<vector<int>, int>, int, decltype(key_hash)>
      umap(r_domain->ids.size(), key_hash);

  for (size_t i=0, n=r_domain->ids.size(); i<n; ++i) {
    pair<vector<int>, int> key;
    key.first = r_domain->preconditions[i];
    std::sort(key.first.begin(), key.first.end());
    key.second = r_domain->effects[i];

    auto entry = umap.find(key);

    if (entry == umap.end()) {
      umap[key] = i;
    } else if (r_domain->costs[i] < r_domain->costs[entry->second]) {
      entry->second = i;
    }
  }

  vector<int> ids;
  vector<int> costs;
  vector<int> precondition_size;
  vector<vector<int> > preconditions;
  vector<int> effects;

  for (auto &v : umap) {
    auto key = v.first;
    auto p = key.first;
    int e = key.second;
    int a = v.second;
    int cost = r_domain->costs[a];

    bool match = false;

    if (p.size() <= 5) {
      int pw_size = (1 << p.size()) - 1;

      // example: p.size() == 2
      // mask: 000, 001, 010, 011, 100, 101, 110, 111
      for (int mask=0; mask<pw_size; ++mask) {
        auto dominate_key = std::make_pair(vector<int>(), e);

        // 1 << i: 001, 010, 100
        for (size_t i=0; i<p.size(); ++i)
          if (mask & (1 << i)) dominate_key.first.push_back(p[i]);

        auto entry = umap.find(dominate_key);

        if (entry != umap.end() && r_domain->costs[entry->second] <= cost) {
          match = true;
          break;
        }
      }
    }

    if (!match) {
      ids.push_back(r_domain->ids[a]);
      costs.push_back(cost);
      precondition_size.push_back(p.size());
      preconditions.push_back(p);
      effects.push_back(e);
    }
  }

  r_domain->ids = ids;
  r_domain->costs = costs;
  r_domain->precondition_size = precondition_size;
  r_domain->preconditions = preconditions;
  r_domain->effects = effects;
}

void InitializeRelaxedDomain(const Domain &domain, RelaxedDomain *r_domain) {
  r_domain->fact_size = domain.fact_size;
  r_domain->goal_size = static_cast<int>(domain.goal.size());

  r_domain->is_goal.resize(domain.fact_size, false);

  vector<int> precondition;

  for (size_t i=0, n=domain.action_size; i<n; ++i) {
    precondition.clear();

    for (auto &v : domain.preconditions[i]) {
      int f = ToFact(domain.fact_offset, v);
      precondition.push_back(f);
    }

    size_t cost = domain.costs[i];
    size_t size = domain.preconditions[i].size();

    for (auto &v : domain.effects[i]) {
      int f = ToFact(domain.fact_offset, v);
      r_domain->ids.push_back(i);
      r_domain->costs.push_back(cost);
      r_domain->precondition_size.push_back(size);
      r_domain->preconditions.push_back(precondition);
      r_domain->effects.push_back(f);
    }
  }

  std::cout << r_domain->ids.size() << " unary operators" << std::endl;

  Simplify(r_domain);

  size_t unary_size = r_domain->ids.size();
  std::cout << "simplified to " << unary_size << std::endl;
  int unary_int = static_cast<int>(unary_size);
  r_domain->action_size = unary_size;

  r_domain->precondition_map.resize(domain.fact_size);

  for (int i=0; i<unary_int; ++i) {
    for (auto f : r_domain->preconditions[i])
      r_domain->precondition_map[f].push_back(i);
  }

  r_domain->effect_map.resize(domain.fact_size);

  for (int i=0; i<unary_int; ++i) {
    int f = r_domain->effects[i];
    r_domain->effect_map[f].push_back(i);
  }

  r_domain->goal.resize(0);

  for (auto v : domain.goal) {
    int f = ToFact(domain.fact_offset, v);
    r_domain->is_goal[f] = true;
    r_domain->goal.push_back(f);
  }

}

} // namespace rwls
