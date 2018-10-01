#ifndef ATOMIC_LTS_H_
#define ATOMIC_LTS_H_

#include <vector>

#include "dtg.h"
#include "sas_plus.h"

namespace pplanner {

class AtomicLTS : public DTG {
 public:
  AtomicLTS() {}

  AtomicLTS(const std::vector<std::vector<int> > &adjacent_matrix,
            int initial, int goal,
            const std::vector<std::vector<std::vector<int> > > &labels,
            const std::vector<bool> &tau) {
    : DTG(adjacent_matrix),
      initial_(initial),
      goal_(goal),
      labels_(labels),
      tau_(tau),
      tau_(tau.size(), 0),
      tau_cost_(tau.size(), 1),
      label_from_(tau.size(), -1),
      label_to_(tau.size(), -1),
      h_star_cache_(labels.size(), std::vector<int>(labels.size(), -1)),
      h_tau_cache_(labels.size(), std::vector<int>(labels.size(), -1)),
      closed_(adjacent_matrix.size(), -1) {
    Init();
  }

  ~AtomicLTS() {}

  int n_labels() const { return tau_.size(); }

  int Initial() const { return initial_; }

  int Goal() const { return goal_; }

  void MinLabel(int s, int t, int *label, int *cost, bool only_tau=false) const;

  int HStar(int start, int goal, bool only_tau=false);

  const std::vector<int>& Labels(int s, int t) const { return labels_[s][t]; }

  int LabelFrom(int l) const { return label_from_[l]; }

  int LabelTo(int l) const { return label_to_[l]; }

  bool IsTauLabel(int l) const { return tau_[l]; }

  void SetTauLabel(int l, int cost) {
    tau_[l] = true;
    tau_cost_[l] = cost;
    ClearTauCache();
  }

 private:
  void Init();

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
  std::vector<std::vector<std::vector<int> > > labels_;
  std::vector<bool> tau_;
  std::vector<int> tau_cost_;
  std::vector<int> label_from_;
  std::vector<int> label_to_;
  std::vector<std::vector<int> > h_star_cache_;
  std::vector<std::vector<int> > h_tau_cache_;
  PQueue open_;
  std::vector<int> closed_;

  static std::shared_ptr<const SASPlus> problem_;
};

std::vector<std::shared_ptr<AtomicLTS> > InitializeLTSs(
    std::shared_ptr<const SASPlus> problem_);

void AddRecursiveTauLabel(std::vector<std::shared_ptr<AtomicLTS> > &ltss);

} // namespace ppalnner

#endif // ATOMIC_LTS_H_
