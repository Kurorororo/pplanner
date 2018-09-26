#ifndef STRONG_STUBBORN_SETS_H_
#define STRONG_STUBBORN_SETS_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class SSSApproximater {
 public:
  SSSApproximater(std::shared_ptr<const SASPlus> problem) : problem_(problem) {
    InitAchievers();
    //InitInterfere();
  }

  ~SSSApproximater() {}

  void ApproximateSSS(const std::vector<int> &state,
                     const std::vector<int> &applicable,
                     std::vector<bool> &sss) const;

 private:
  void InitAchievers();

  void InitInterfere();

  void FDOrderGoal(const std::vector<int> &state, int *goal_var,
                   int *goal_value) const;

  void FDOrderPrecondition(const std::vector<int> &state, int a, int *goal_var,
                           int *goal_value) const;

  bool MutexInterfere(int a, int b) const;

  std::vector<std::vector<int> > to_achievers_;
  std::vector<std::vector<int> > to_interfere_;
  std::shared_ptr<const SASPlus> problem_;
};

} // namespace pplanner

#endif // STRONG_STUBBORN_SETS_H_
