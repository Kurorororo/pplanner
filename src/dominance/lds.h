#ifndef LDS_H_
#define LDS_H_

#include <memory>
#include <vector>

#include "sas_plus.h"
#include "dominance/lts.h"

namespace pplanner {

class LDS {
 public:
  LDS(std::shared_ptr<const SASPlus> problem)
    : r_(problem->n_variables()),
      ltss_(InitializeLTSs(problem)) { Init(); }

  ~LDS() {}

  bool Dominance(int i, int s, int t) const { return r_[i][s][t]; }

  bool Dominance(const std::vector<int> &s, const std::vector<int> &t) const;

  bool LabelDominance(int j, int l, int l_p) const;

  bool TransisionDominance(int i, int s, int s_p, int t, int t_p) const;

  void Dump() const;

 private:
  void Init() { ComputeRelations(); }

  void ComputeRelations();

  void InitRelations();

  bool Ok(int i, int s, int t) const;

  std::vector<std::vector<std::vector<bool> > > r_;
  std::vector<std::shared_ptr<AtomicLTS> > ltss_;
};

} // namespace pplanner

#endif // LDS_H_
