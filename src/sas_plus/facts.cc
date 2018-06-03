#include "sas_plus/facts.h"

#include <iostream>

using std::vector;

namespace pplanner {

int Facts::AddVariable(const vector<std::string> &predicates) {
  predicates_.insert(predicates_.end(), predicates.begin(), predicates.end());
  int range = static_cast<int>(predicates.size());
  size_ += range;
  ranges_.push_back(range);
  offsets_.push_back(offsets_.back() + range);

  return static_cast<int>(n_variables_++);
}

void Facts::Dump() const {
  int n = static_cast<int>(n_variables());
  std::cout << n << " variables" << std::endl;

  for (int i=0; i<n; ++i)
    std::cout << "var" << i << " < " << VarRange(i) << std::endl;

  std::cout << std::endl;
  std::cout << "Fact offsets" << std::endl;

  for (int i=0; i<n; ++i)
    std::cout << "var" << i << "=0: " << VarBegin(i) << std::endl;

  std::cout << std::endl;
  int n_facts = static_cast<int>(size());
  std::cout << n_facts << "facts" <<  std::endl;

  for (int i=0; i<n; ++i) {
    for (int j=0; j<VarRange(i); ++j) {
      int f = Fact(i, j);
      std::cout << "Fact" << f << " predicate ";
      std::cout << Predicate(i, j) << std::endl;
    }
  }
}

} // namespace pplanner
