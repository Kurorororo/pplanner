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
      adjacent_lists_(dtg.adjacent_lists_) {}

  explicit DTG(const std::vector<std::vector<int> > &adjacent_matrix)
    : adjacent_matrix_(adjacent_matrix) {
    InitTransitionLists(adjacent_matrix);
  }

  ~DTG() {}

  DTG &operator=(const DTG &dtg) {
    adjacent_matrix_ = dtg.adjacent_matrix_;
    adjacent_lists_ = dtg.adjacent_lists_;

    return *this;
  }

  size_t n_nodes() const { return adjacent_matrix_.size(); }

  void RemoveNode(int value);

  void SoftRemoveNode(int value) { deleted_.insert(value); }

  void RecoverSoftDelete() { deleted_.clear(); }

  bool IsConnected(int start, int goal, int ignore=-1);

  double GreedyCut(std::vector<int> &cut) const;

  double SparsestCut(std::vector<int> &cut) const;

  void Dump() const;

 private:
  void InitTransitionLists(
      const std::vector<std::vector<int> > &adjacent_matrix);

  bool RecursiveIsConnected(int i, int goal);

  double RecursiveSparsestCut(int value, double answer,
                              std::vector<int> &cut) const;

  double CalculateSparsity(const std::vector<int> &cut) const;

  double CalculateUpperBound(const std::vector<int> &cut) const;

  std::vector<std::vector<int> > adjacent_matrix_;
  std::vector<std::vector<int> > adjacent_lists_;
  std::unordered_set<int> closed_;
  std::unordered_set<int> deleted_;
};

std::vector<DTG> InitializeDTGs(std::shared_ptr<const SASPlus> problem);

} // namespace pplanner

#endif // DTG_H_
