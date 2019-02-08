#ifndef HEURISTIC_H_
#define HEURISTIC_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "multithread_search/search_node.h"

namespace pplanner {

class Heuristic {
 public:
  virtual ~Heuristic() = 0;

  virtual int Evaluate(const std::vector<int> &state,
                       std::shared_ptr<SearchNode> node) = 0;

  virtual int Evaluate(const std::vector<int> &state,
                       const std::vector<int> &applicable,
                       std::unordered_set<int> &preferred,
                       std::shared_ptr<SearchNode> node) = 0;
};

int Evaluate(const std::vector<std::shared_ptr<Heuristic> > evaluators,
             const std::vector<int> &state, std::shared_ptr<SearchNode> node,
             std::vector<int> &values);

} // namespace pplanner

#endif // HEURISTIC_H_
