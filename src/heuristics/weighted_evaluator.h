#ifndef WEIGHTED_EVALUATOR_H_
#define WEIGHTED_EVALUATOR_H_

#include "evaluator.h"
#include "sas_plus.h"
#include "search_graph.h"

#include <boost/property_tree/ptree.hpp>

namespace pplanner {

std::shared_ptr<Evaluator> EvaluatorFactory(
    std::shared_ptr<const SASPlus> problem,
    std::shared_ptr<SearchGraph> graph,
    std::shared_ptr<Evaluator> friend_evaluator,
    const boost::property_tree::ptree &pt);

class WeightedEvaluator : public Evaluator {
 public:
  WeightedEvaluator() : weight_(1), cache_(-1), search_graph_(nullptr) {}

  WeightedEvaluator(int weight, std::shared_ptr<const SASPlus> problem,
                    std::shared_ptr<SearchGraph> search_graph,
                    std::shared_ptr<Evaluator> friend_evaluator,
                    const boost::property_tree::ptree &pt)
    : weight_(weight), cache_(-1), search_graph_(search_graph) {
    heuristic_ = EvaluatorFactory(problem, search_graph, friend_evaluator, pt);
  }

  ~WeightedEvaluator() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    int h = heuristic_->Evaluate(state, node);
    cache_ = h;

    return search_graph_->Cost(node) + weight_ * h;
  }

  int Evaluate(const std::vector<int> &state, int node, int parent) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    int h = heuristic_->Evaluate(state, node, applicable, preferred);
    cache_ = h;

    return search_graph_->Cost(node) + weight_ * h;
  }

  int Evaluate(const std::vector<int> &state, int node, int parent,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node, applicable, preferred);
  }

  int HeuristicCache() const { return cache_; }

 private:
  int weight_;
  int cache_;
  std::shared_ptr<SearchGraph> search_graph_;
  std::shared_ptr<Evaluator> heuristic_;
};

class WeightedHeuristicCache : public Evaluator {
 public:
  WeightedHeuristicCache() : evaluator_(nullptr) {}

  WeightedHeuristicCache(std::shared_ptr<WeightedEvaluator> evaluator)
    : evaluator_(evaluator) {}

  int Evaluate(const std::vector<int> &state, int node) override {
    return evaluator_->HeuristicCache();
  }

  int Evaluate(const std::vector<int> &state, int node, int parent) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node, applicable, preferred);
  }

 private:
  std::shared_ptr<WeightedEvaluator> evaluator_;
};

} // namespace pplanner

#endif // WEIGHTED_EVALUATOR_H_

