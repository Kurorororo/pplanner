#ifndef QDF_H_
#define QDF_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class QDF {
 public:
  QDF(std::shared_ptr<const SASPlus> problem) : d_(problem->n_variables()) {
    Init(problem);
  }

  ~QDF() {}

  int Dominance(int i, int s, int t) const { return d_[i][s][t]; }

  int Dominance(const std::vector<int> &s, const std::vector<int> &t) const;

  int FQLD(const std::vector<std::shared_ptr<LTS> > &ltss, int i, int s, int t)
    const;

 private:
  void Init(std::shared_ptr<const SASPlus> problem, int limit=10);

  std::vector<std::vector<std::vector<int> > > d_;
}


} // namespace pplanner

#endif // QDF_H_
