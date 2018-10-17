#ifndef STRONG_STUBBORN_SETS_H_
#define STRONG_STUBBORN_SETS_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class SSSApproximater {
 public:
  SSSApproximater(std::shared_ptr<const SASPlus> problem)
    : to_interfere_(problem->n_actions(), std::vector<int>()),
      interfere_computed_(problem->n_actions(), false),
      problem_(problem) {
    InitAchievers();
  }

  ~SSSApproximater() {}

  void ApproximateSSS(const std::vector<int> &state,
                     const std::vector<int> &applicable,
                     std::vector<bool> &sss);

 private:
  void InitAchievers();

  void FDOrderGoal(const std::vector<int> &state, int *goal_var,
                   int *goal_value) const;

  void FDOrderPrecondition(const std::vector<int> &state, int a, int *goal_var,
                           int *goal_value) const;

  void ComputeInterfere(int a);

  bool AreInterfere(int a, int b) const {
    if (a == b || PreconditionsMutex(a, b)) return false;

    return Disable(a, b) || Disable(b, a) || Conflict(a, b);
  }

  bool PreconditionsMutex(int a, int b) const;

  bool Disable(int a, int b) const;

  bool Conflict(int a, int b) const;

  std::vector<std::vector<int> > to_achievers_;
  std::vector<std::vector<int> > to_interfere_;
  std::vector<bool> interfere_computed_;
  std::shared_ptr<const SASPlus> problem_;
};

} // namespace pplanner

#endif // STRONG_STUBBORN_SETS_H_
