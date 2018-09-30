#ifndef DTG_H_
#define DTG_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class DTG {
 public:
  DTG() {}

  explicit DTG(const DTG &dtg)
    : adjacent_matrix_(dtg.adjacent_matrix_),
      adjacent_lists_(dtg.adjacent_lists_),
      in_degrees_(dtg.in_degrees_),
      out_degrees_(dtg.out_degrees_),
      nodes_by_degree_(dtg.nodes_by_degree_) {}

  explicit DTG(const std::vector<std::vector<int> > &adjacent_matrix)
    : adjacent_matrix_(adjacent_matrix) {
    Init(adjacent_matrix);
  }

  ~DTG() {}

  DTG &operator=(const DTG &dtg) {
    adjacent_matrix_ = dtg.adjacent_matrix_;
    adjacent_lists_ = dtg.adjacent_lists_;

    return *this;
  }

  int n_nodes() const { return adjacent_matrix_.size(); }

  void RemoveNode(int value);

  void SoftRemoveNode(int value) { deleted_.insert(value); }

  void RecoverSoftDelete() { deleted_.clear(); }

  bool IsAdjacent(int i, int j) const { return adjacent_matrix_[i][j] > 0; }

  bool IsConnected(int start, int goal, int ignore=-1);

  int InDegree(int i) const { return in_degrees_[i]; }

  int OutDegree(int i) const { return out_degrees_[i]; }

  int Degree(int i) const { return InDegree(i) + OutDegree(i); }

  double GreedyCut(std::vector<int> &cut) const;

  double SparsestCut(std::vector<int> &cut, int max_expansion=100000) const;

  const std::vector<int>& AdjacentList(int i) const {
    return adjacent_lists_[i];
  }

  void Dump() const;

 private:
  void Init(const std::vector<std::vector<int> > &adjacent_matrix);

  bool RecursiveIsConnected(int i, int goal);

  double RecursiveSparsestCut(int value, double answer,
                              std::vector<int> &cut, int *count) const;

  double CalculateSparsity(const std::vector<int> &cut) const;

  double CalculateUpperBound(const std::vector<int> &cut) const;

  std::vector<std::vector<int> > adjacent_matrix_;
  std::vector<std::vector<int> > adjacent_lists_;
  std::vector<int> in_degrees_;
  std::vector<int> out_degrees_;
  std::vector<int> nodes_by_degree_;
  std::unordered_set<int> closed_;
  std::unordered_set<int> deleted_;
};

std::vector<std::shared_ptr<DTG> > InitializeDTGs(
    std::shared_ptr<const SASPlus> problem);

} // namespace pplanner

#endif // DTG_H_
