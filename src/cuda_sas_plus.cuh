#ifndef CUDA_SAS_PLUS_H_
#define CUDA_SAS_PLUS_H_

#include <memory>

#include "sas_plus.h"

namespace pplanner {

struct CudaSASPlus {
  int metric;
  int n_variables;
  int n_goal_facts;
  int n_actions;
  bool use_conditional;
  int *var_offsets;
  int *goal_vars;
  int *goal_values;
  int *action_costs;
  int *effect_offsets;
  int *effect_vars;
  int *effect_values;
  int *effect_condition_offsets_1;
  int *effect_condition_offsets_2;
  int *effect_condition_vars;
  int *effect_condition_values;
  int *conditional_effect_vars;
  int *conditional_effect_values;
};

extern __constant__ CudaSASPlus cuda_problem;

__device__
inline int Fact(const CudaSASPlus &problem, int var, int value) {
  return problem.var_offsets[var] + value;
}

__device__
bool IsGoal(const CudaSASPlus &problem, const int *state);

__device__
void ApplyEffect(const CudaSASPlus &problem, int i, const int *state,
                 int *child);

void InitCudaSASPlus(std::shared_ptr<const SASPlus> problem,
                     CudaSASPlus *cuda_problem);

void FreeCudaSASPlus(CudaSASPlus *problem);

} // namespace pplanner

#endif // CUDA_SAS_PLUS_H_
