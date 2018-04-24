#include "ff_add.h"

#include <algorithm>
#include <limits>

using std::vector;
using std::unordered_set;

namespace rwls {

void InitializeRpgSet(const Domain &domain, RpgSet *rpg) {
  rpg->plan.resize(domain.action_size);
  rpg->marked.resize(domain.fact_size);
}

void ResetRpgSet(RpgSet *rpg) {
  std::fill(rpg->plan.begin(), rpg->plan.end(), false);
  std::fill(rpg->marked.begin(), rpg->marked.end(), false);
}

void SetPlan(int g, const RelaxedDomain &r_domain,
             const AdditiveTable &table, RpgSet *rpg) {
  if (rpg->marked[g]) return;

  rpg->marked[g] = true;
  int unary_a = table.best_support[g];
  if (unary_a == -1) return;

  for (auto p : r_domain.preconditions[unary_a])
    SetPlan(p, r_domain, table, rpg);

  int a = r_domain.ids[unary_a];
  rpg->plan[a] = true;
}

int PlanCost(const Domain &domain, const RelaxedDomain &r_domain,
             const AdditiveTable &table, RpgSet *rpg) {
  ResetRpgSet(rpg);

  for (auto g : r_domain.goal)
    SetPlan(g, r_domain, table, rpg);

  int h = 0;

  for (size_t i=0, n=rpg->plan.size(); i<n; ++i)
    if (rpg->plan[i]) h += domain.costs[i];

  return h;
}

int PlanCost(const Domain &domain, const RelaxedDomain &r_domain,
             const AdditiveTable &table, const vector<int> &applicable,
             unordered_set<int> &preferred, RpgSet *rpg) {
  preferred.clear();
  ResetRpgSet(rpg);

  for (auto g : r_domain.goal)
    SetPlan(g, r_domain, table, rpg);

  for (auto a : applicable)
    if (rpg->plan[a]) preferred.insert(a);

  int h = 0;

  for (size_t i=0, n=rpg->plan.size(); i<n; ++i)
    if (rpg->plan[i]) h += domain.costs[i];

  return h;
}

} // namespace rwls
