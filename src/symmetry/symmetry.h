#ifndef SYMMETRY_H_
#define SYMMETRY_H_

#include <memory>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class SymmetryManager {
 public:
  SymmetryManager(std::shared_ptr<const SASPlus> problem) : problem_(problem) {
    Init(problem);
  }

  ~SymmetryManager() {}

  void ToCanonical(const std::vector<int> &state, std::vector<int> &canonical)
    const;

  void ToCanonical(const std::vector<int> &state, std::vector<int> &canonical,
                   std::vector<int> &generators) const;

  void AddGenerator(const unsigned int n, const unsigned int *aut);

  void Permutate(int i, const std::vector<int> &state,
                 std::vector<int> &permutated) const;

  void InversePermutate(int i, const std::vector<int> &state,
                        std::vector<int> &permutated) const;

  void Dump() const;

 private:
  void AddInverseGenerators();

  void Init(std::shared_ptr<const SASPlus> problem);


  std::vector<int> goal_;
  std::vector<unsigned int> var_to_id_;
  std::vector<std::vector<unsigned int> > value_to_id_;
  std::vector<int> id_to_var_;
  std::vector<int> id_to_value_;
  std::vector<std::vector<int> > var_permutations_;
  std::vector<std::vector<std::vector<int> > > value_permutations_;
  std::vector<std::vector<int> > inverse_var_permutations_;
  std::vector<std::vector<std::vector<int> > > inverse_value_permutations_;
  std::shared_ptr<const SASPlus> problem_;
};


} // namespace pplanner

#endif // SYMMETRY_H_
