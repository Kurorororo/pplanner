#ifndef SYMMETRY_H_
#define SYMMETRY_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class SymmetryManager {
 public:
  SymmetryManager(std::shared_ptr<const SASPlus> problem) { Init(problem); }

  ~SymmetryManager();

  void ToCanonical(const std::vector<int> &state, std::vector<int> &canonical)
    const;

  void AddGenerator(const unsigned int n, const unsigned int *aut);

  void Dump() const;

 private:
  void Init(std::shared_ptr<const SASPlus> problem);

  void Permutation(const std::vector<unsigned int> &permutation,
                   const std::vector<int> &state, std::vector<int> &permutated);

  int LexicalOrder(const std::vector<int> &state) const;

  std::vector<int> goal_;
  std::vector<int> prefix_;
  std::vector<unsigned int> var_to_id_;
  std::vector<std::vector<unsigned int> > value_to_id_;
  std::vector<int> id_to_var_;
  std::vector<int> id_to_value_;
  std::vector<std::vector<unsigned int> > generators_;
};


} // namespace pplanner

#endif // SYMMETRY_H_
