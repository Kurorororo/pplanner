#ifndef EVALUATOR_H_
#define EVALUATOR_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "search_graph.h"
#include "search_node.h"

namespace pplanner {

class Evaluator {
 public:
  virtual ~Evaluator() = 0;

  virtual int Evaluate(const std::vector<int> &state, int node) = 0;

  virtual int Evaluate(const std::vector<int> &state, int node,
                       const std::vector<int> &applicable,
                       std::unordered_set<int> &preferred) = 0;

  // For MPI
  virtual int Evaluate(const std::vector<int> &state, int node, int parent) = 0;

  virtual int Evaluate(const std::vector<int> &state, int node, int parent,
                       const std::vector<int> &applicable,
                       std::unordered_set<int> &preferred) = 0;

  // For multithread
  virtual int Evaluate(const std::vector<int> &state,
                       std::shared_ptr<SearchNode> node) = 0;

  virtual int Evaluate(const std::vector<int> &state,
                       std::shared_ptr<SearchNode> node,
                       const std::vector<int> &applicable,
                       std::unordered_set<int> &preferred) = 0;

  // For random walk
  virtual int Evaluate(const std::vector<int> &state) = 0;

  virtual int Evaluate(const std::vector<int> &state,
                       const std::vector<int> &applicable,
                       std::unordered_set<int> &preferred) = 0;

  virtual void UpdateBest() = 0;

  virtual void LocalRestart() = 0;

  virtual void GlobalRestart() = 0;

  virtual void CopyBestToSearchGraph(int node,
                                     std::shared_ptr<SearchGraph> graph) = 0;
};

int Evaluate(const std::vector<std::shared_ptr<Evaluator> > evaluators,
             const std::vector<int> &state, int node, std::vector<int> &values);

}  // namespace pplanner

#endif  // EVALUATOR_H_
