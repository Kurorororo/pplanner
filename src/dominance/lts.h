#ifndef ATOMIC_LTS_H_
#define ATOMIC_LTS_H_

#include <vector>

#include "sas_plus.h"

namespace pplanner {

class AtomicLTS {
 public:
  AtomicLTS(int initial, int goal, const std::vector<int> &label_to,
            const std::vector<bool> &is_tau_label,
            const std::vector<std::vector<int> > &labels) {
    : initial_(initial),
      goal_(goal),
      label_to_(label_to),
      tau_cost_(is_tau_label.size(), kInfinity),
      labels_(labels),
      h_star_cache_(labels.size(), std::vector<int>(labels.size(), -1)),
      h_tau_cache_(labels.size(), std::vector<int>(labels.size(), -1)),
      closed_(adjacent_matrix.size(), -1) { InitTauCost(is_tau_label); }

  ~AtomicLTS() {}

  int n_states() const { return labels_.size(); }

  int n_labels() const { return is_tau_label_.size(); }

  int initial_node() const { return initial_; }

  int goal_node() const { return goal_; }

  // -1 : noop

  int LabelFrom(int l) const { return l == -1 ? -1 : label_from_[l]; }

  int LabelTo(int l) const { return l == -1 ? -1 : label_to_[l]; }

  int LabelCost(int l) const { return l == -1 ? 0 : 1; }

  int TauLabelCost(int l) const { return tau_cost_[l]; }

  bool IsTauLabel(int l) const { return l != -1 && tau_[l] != kInfinity; }

  void SetTauLabel(int l, int cost) {
    tau_cost_[l] = cost;
    ClearTauCache();
  }

  int ShortestPathCost(int start, int goal, bool only_tau=false);

  const std::vector<int>& Labels(int s) const { return labels_[s]; }

  static constexpr kInfinity = std::numeric_limits<int>::max();

 private:
  void InitTauCost(const std::vector<bool> &is_tau_label);

  void ClearTauCache();

  struct FirstGreater {
    bool operator()(const std::pair<int, int> &a,
                    const std::pair<int, int> &b) {
      return a.first > b.first;
    }
  };

  using PQueue = std::priority_queue<std::pair<int, int>,
                                     std::vector<std::pair<int, int> >,
                                     FirstGreater>;
  int initial_;
  int goal_;
  std::vector<int> label_from_;
  std::vector<int> label_to_;
  std::vector<int> tau_cost_;
  std::vector<std::vector<int> > labels_;
  std::vector<std::vector<int> > h_star_cache_;
  std::vector<std::vector<int> > h_tau_cache_;
  PQueue open_;
  std::vector<int> closed_;
};

std::vector<std::shared_ptr<AtomicLTS> > InitializeLTSs(
    std::shared_ptr<const SASPlus> problem_);

void AddRecursiveTauLabel(std::vector<std::shared_ptr<AtomicLTS> > &ltss);

} // namespace ppalnner

#endif // ATOMIC_LTS_H_
