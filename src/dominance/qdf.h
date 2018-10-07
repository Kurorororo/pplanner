#ifndef QDF_H_
#define QDF_H_

#include <memory>
#include <vector>

#include "sas_plus.h"
#include "dominance/lts.h"

namespace pplanner {

class QDF {
 public:
  QDF(std::shared_ptr<const SASPlus> problem, int limit=10)
    : d_(problem->n_variables()),
      r_(problem->n_variables()),
      ltss_(InitializeLTSs(problem)) { Init(problem, limit); }

  ~QDF() {}

  int Dominance(int i, int s, int t) const { return d_[i][s][t]; }

  int Dominance(const std::vector<int> &s, const std::vector<int> &t) const;

  int LabelDominance(int j, int l, int l_p) const;

  int FQLD(int i, int s, int t) const;

  bool QualitativeDominance(int i, int s, int t) const { return r_[i][s][t]; }

  bool QualitativeDominance(const std::vector<int> &s,
                            const std::vector<int> &t) const;

  void Dump() const;

 private:
  void Init(std::shared_ptr<const SASPlus> problem, int limit) {
    ComputeRelations();
  }

  void ComputeFunctions(std::shared_ptr<const SASPlus> problem, int limit);

  void InitFunctions(std::shared_ptr<const SASPlus> problem, int limit);

  int FQLDInner(int i, int s, int t, int l, int s_p) const;

  void ComputeRelations();

  void InitRelations();

  bool Ok(int i, int s, int t) const;

  std::vector<std::vector<std::vector<int> > > d_;
  std::vector<std::vector<std::vector<bool> > > r_;
  std::vector<std::shared_ptr<AtomicLTS> > ltss_;
};

} // namespace pplanner

#endif // QDF_H_
