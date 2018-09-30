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
            const std::vector<bool> &tau,
            std::shared_ptr<const SASPlus> problem)
    : DTG(adjacent_matrix),
      initial_(initial),
      goal_(goal),
      labels_(labels),
      tau_(tau)
      label_from_(problem->n_actions() -1),
      label_to_(problem->n_actions(), -1),
      problem_(problem),
      h_star_cache_(labels.size(), std::vector<int>(labels.size(), -1)),
      closed_(adjacent_matrix.size(), -1) {
    Init()
  }

  ~AtomicLTS() {}

  int Initial() const { return initial_; }

  int Goal() const { return goal_; }

  int HStar(int start, bool only_tau=false) const;

  void MinLabel(int s, int t, int *label, int *cost, bool only_tau=false) const;

  const std::vector<int>& Labels(int s, int t) const { return labels_[s][t]; }

  int LabelFrom(int l) const { return label_from_[l]; }

  int LabelTo(int l) const { return label_to_[l]; }

 private:
  void Init();

  struct FirstGreater {
    bool operator()(const std::pair<int, int> &a, const std::pair<int, int> &b) {
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
  std::vector<int> label_from_;
  std::vector<int> label_to_;
  std::shared_ptr<const SASPlus> problem_;
  mutable std::vector<int> h_star_cache_;
  mutable PQueue open_;
  mutable std::vector<int> closed_;
};

std::vector<std::shared_ptr<AtomicLTS> > InitializeLTSs(
    std::shared_ptr<const SASPlus> problem_);

} // namespace ppalnner

#endif // ATOMIC_LTS_H_
