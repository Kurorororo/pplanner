#ifndef HEURISTIC_H_
#define HEURISTIC_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "multithread_search/search_node.h"

namespace pplanner {

template<typename T>
class Heuristic {
 public:
  virtual ~Heuristic() = 0;

  virtual int Evaluate(const std::vector<int> &state, T node) = 0;

  virtual int Evaluate(const std::vector<int> &state,
                       const std::vector<int> &applicable,
                       std::unordered_set<int> &preferred, T node) = 0;
};

template<typename T>
Heuristic<T>::~Heuristic() {}

template<typename T>
int Evaluate(const std::vector<std::shared_ptr<Heuristic<T> > > evaluators,
             const std::vector<int> &state, T node, std::vector<int> &values) {
  values.clear();

  for (auto e : evaluators) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}

} // namespace pplanner

#endif // HEURISTIC_H_
