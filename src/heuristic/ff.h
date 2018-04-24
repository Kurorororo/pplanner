#ifndef FF_H_
#define FF_H_

#include <limits>
#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "heuristic/graphplan.h"
#include "heuristic/heuristic.h"

namespace rwls {

class FF : public HeuristicInterface<FF> {
 public:
  void Initialize(const Domain &domain) {
    state_.resize(domain.variables_size);
    InitializeSchema(domain, &schema_);
    InitializeGraph(domain, schema_, &graph_);
  }

  inline int operator()(const State &state, const Domain &domain,
                        const std::vector<int> &applicable,
                        std::unordered_set<int> &preferred) {
    if (applicable.empty()) return std::numeric_limits<int>::max();

    return GraphplanCost(state, domain, schema_, &graph_, preferred);
  }

  int operator()(int parent, int node, const State &state,
                 const Domain &domain) {
    return GraphplanCost(state, domain, schema_, &graph_);
  }

  int operator()(const State &state, const Domain &domain) {
    return GraphplanCost(state, domain, schema_, &graph_);
  }

 private:
  State state_;
  GraphSchema schema_;
  PlanningGraph graph_;
};

class FFPreferring : public Preferring {
 public:
  FFPreferring() {}

  FFPreferring(const Domain &domain) {
    InitializeSchema(domain, &schema_);
    InitializeGraph(domain, schema_, &graph_);
  }

  void extract(const State &state, const Domain &domain,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) {
    GraphplanCost(state, domain, schema_, &graph_, preferred);
  }

 private:
  GraphSchema schema_;
  PlanningGraph graph_;
};

} // namespace rwls

#endif // FF_H_
