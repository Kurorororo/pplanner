#ifndef ABSTRACT_GRAPH_H_
#define ABSTRACT_GRAPH_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class AbstractGraph {
 public:
  AbstractGraph(std::shared_ptr<const SASPlus> problem,
                const std::vector<int> &vars);

  double Delta() const {
    return static_cast<double>(max_out_degree_) / static_cast<double>(n_nodes_);
  }

  void BuildInferenceScope();

  const std::vector<int>& InferenceScope(int i) const {
    return inference_scope_[i];
  }

  int n_nodes() const { return n_nodes_; }

  int max_out_degree() const { return max_out_degree_; }

  std::vector<int> vars() const { return vars_; }

  void Dump() const;

 private:
  bool IsActionRelevant(std::shared_ptr<const SASPlus> problem, int action)
    const;

  void ApplyEffect(std::shared_ptr<const SASPlus> problem, int action,
                   const std::vector<int> &state, std::vector<int> &child)
    const;

  void AddEdge(std::shared_ptr<const SASPlus> problem, int action,
               const std::vector<std::vector<int> > &possible_values, int index,
               std::vector<int> &state);

  void ConstructGraph(std::shared_ptr<const SASPlus> problem);

  int n_nodes_;
  int max_out_degree_;
  std::vector<int> vars_;
  std::vector<int> ranges_;
  std::vector<std::vector<bool> > matrix_;
  std::vector<std::vector<int> > to_parents_;
  std::vector<std::vector<int> > to_children_;
  std::vector<std::vector<int> > inference_scope_;
};

std::shared_ptr<AbstractGraph> GreedySelect(
    std::shared_ptr<const SASPlus> problem,
    const std::vector<int> &candidates,
    const std::vector<int> &selected,
    int max_n_nodes);

std::shared_ptr<AbstractGraph> ConstructByDTG(
    std::shared_ptr<const SASPlus> problem,
    int max_n_nodes);

}

#endif // ABSTRACT_GRAPH_H_
