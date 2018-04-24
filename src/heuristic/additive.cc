#include "additive.h"

#include <algorithm>

namespace rwls {

using std::vector;

void InitializeAdditiveTable(const RelaxedDomain &domain,
                             AdditiveTable *table) {
  table->op_cost.resize(domain.action_size);
  table->prop_cost.resize(domain.fact_size);
  table->best_support.resize(domain.fact_size);
  table->precondition_counter.resize(domain.action_size);
}

void MayPush(int f, int a, PQueue &q, AdditiveTable *table) {
  int op_c = table->op_cost[a];

  if (op_c < table->prop_cost[f]) {
    table->best_support[f] = a;
    table->prop_cost[f] = op_c;
    q.push(std::make_pair(op_c, f));
  }
}


void InitializeQueue(const vector<int> &state, const RelaxedDomain &domain,
                     PQueue &q, AdditiveTable *table) {
  q = PQueue();

  table->goal_counter = domain.goal_size;

  std::fill(table->best_support.begin(), table->best_support.end(), -1);
  std::fill(table->prop_cost.begin(), table->prop_cost.end(),
            std::numeric_limits<int>::max());

  for (size_t i=0, n=domain.action_size; i<n; ++i) {
    table->precondition_counter[i] = domain.precondition_size[i];
    table->op_cost[i] = domain.costs[i];

    if (table->precondition_counter[i] == 0)
      MayPush(domain.effects[i], i, q, table);
  }

  for (auto f : state) {
    table->prop_cost[f] = 0;
    q.push(std::make_pair(0, f));
  }
}

void GeneralizedDijkstra(const vector<int> &state, const RelaxedDomain &domain,
                         PQueue &q, AdditiveTable *table) {
  InitializeQueue(state, domain, q, table);

  while (!q.empty()) {
    auto top = q.top();
    q.pop();

    int c = top.first;
    int f = top.second;

    if (table->prop_cost[f] < c) continue;
    if (domain.is_goal[f] && --table->goal_counter == 0) return;

    for (auto a : domain.precondition_map[f]) {
      table->op_cost[a] += c;

      if (--table->precondition_counter[a] == 0)
        MayPush(domain.effects[a], a, q, table);
    }
  }
}

int AdditiveCost(const vector<int> &goal, const AdditiveTable &table) {
  int h = 0;

  for (auto g : goal) {
    int cost = table.prop_cost[g];
    if (cost == std::numeric_limits<int>::max()) return cost;

    h += cost;
  }

  return h;
}

} // namespace rwls
