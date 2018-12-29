#include "cuda_sas_plus.cuh"

#include "cuda_common/cuda_check.cuh"

namespace pplanner {

__constant__ CudaSASPlus cuda_problem;

__device__
void ApplyEffect(const CudaSASPlus &problem, int i, const int *state,
                 int *child) {
  memcpy(child, state, problem.n_variables * sizeof(int));

  if (problem.use_conditional) {
    int b1 = problem.effect_condition_offsets_1[i];
    int e1 = problem.effect_condition_offsets_1[i + 1];

    for (int j = b1; j < e1; ++j) {
      int b2 = problem.effect_condition_offsets_2[j];
      int e2 = problem.effect_condition_offsets_2[j + 1];
      bool all = true;

      for (int k = b2; k < e2; ++k) {
        int var = problem.effect_condition_vars[k];
        int value = problem.effect_condition_values[k];

        if (state[var] != value) {
          all = false;
          break;
        }
      }

      if (all) {
        int var = problem.conditional_effect_vars[j];
        int value = problem.conditional_effect_values[j];
        child[var] = value;
      }
    }
  }

  int b = problem.effect_offsets[i];
  int e = problem.effect_offsets[i + 1];

  for (int j = b; j < e; ++j)
    child[problem.effect_vars[j]] = problem.effect_values[j];
}

__device__
bool IsGoal(const CudaSASPlus &problem, const int *state) {
  for (int i = 0; i < problem.n_goal_facts; ++i)
    if (state[problem.goal_vars[i]] != problem.goal_values[i]) return false;

  return true;
}

void InitConditionalEffects(std::shared_ptr<const SASPlus> problem,
                            CudaSASPlus *cuda_problem) {
  if (problem->use_conditional()) {
    CudaMallocAndCopy((void**)&cuda_problem->effect_condition_offsets_1,
                      problem->effect_condition_offsets_1_data(),
                      (problem->n_actions() + 1) * sizeof(int));
    CudaMallocAndCopy((void**)&cuda_problem->effect_condition_offsets_2,
                      problem->effect_condition_offsets_2_data(),
                      problem->effect_condition_offsets_2_size() * sizeof(int));
    CudaMallocAndCopy((void**)&cuda_problem->effect_condition_vars,
                      problem->effect_condition_vars_data(),
                      problem->effect_conditions_size() * sizeof(int));
    CudaMallocAndCopy((void**)&cuda_problem->effect_condition_values,
                      problem->effect_condition_values_data(),
                      problem->effect_conditions_size() * sizeof(int));
    CudaMallocAndCopy((void**)&cuda_problem->conditional_effect_vars,
                      problem->conditional_effect_vars_data(),
                      problem->conditional_effects_size() * sizeof(int));
    CudaMallocAndCopy((void**)&cuda_problem->conditional_effect_values,
                      problem->conditional_effect_values_data(),
                      problem->conditional_effects_size() * sizeof(int));
  } else {
    cuda_problem->effect_condition_offsets_1 = nullptr;
    cuda_problem->effect_condition_offsets_2 = nullptr;
    cuda_problem->effect_condition_vars = nullptr;
    cuda_problem->effect_condition_values = nullptr;
    cuda_problem->conditional_effect_vars = nullptr;
    cuda_problem->conditional_effect_values = nullptr;
  }
}

void InitCudaSASPlus(std::shared_ptr<const SASPlus> problem,
                     CudaSASPlus *cuda_problem) {
  cuda_problem->metric = problem->metric();
  cuda_problem->n_variables = problem->n_variables();
  cuda_problem->n_goal_facts = problem->n_goal_facts();
  cuda_problem->n_actions = problem->n_actions();
  cuda_problem->use_conditional = problem->use_conditional();

  CudaMallocAndCopy((void**)&cuda_problem->var_offsets,
                    problem->var_offsets_data(),
                    (problem->n_variables() + 1) * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_problem->goal_vars,
                    problem->goal_vars_data(),
                    problem->n_goal_facts() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_problem->goal_values,
                    problem->goal_values_data(),
                    problem->n_goal_facts() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_problem->action_costs,
                    problem->action_costs_data(),
                    problem->n_actions() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_problem->effect_offsets,
                    problem->effect_offsets_data(),
                    (problem->n_actions() + 1) * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_problem->effect_vars,
                    problem->effect_vars_data(),
                    problem->effects_size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_problem->effect_values,
                    problem->effect_values_data(),
                    problem->effects_size() * sizeof(int));

  InitConditionalEffects(problem, cuda_problem);
}

void FreeCudaSASPlus(CudaSASPlus *problem) {
  CUDA_CHECK(cudaFree(problem->var_offsets));
  CUDA_CHECK(cudaFree(problem->goal_vars));
  CUDA_CHECK(cudaFree(problem->goal_values));
  CUDA_CHECK(cudaFree(problem->action_costs));
  CUDA_CHECK(cudaFree(problem->effect_offsets));
  CUDA_CHECK(cudaFree(problem->effect_vars));
  CUDA_CHECK(cudaFree(problem->effect_values));

  if (problem->use_conditional) {
    CUDA_CHECK(cudaFree(problem->effect_condition_offsets_1));
    CUDA_CHECK(cudaFree(problem->effect_condition_offsets_2));
    CUDA_CHECK(cudaFree(problem->effect_condition_vars));
    CUDA_CHECK(cudaFree(problem->effect_condition_values));
    CUDA_CHECK(cudaFree(problem->conditional_effect_vars));
    CUDA_CHECK(cudaFree(problem->conditional_effect_values));
  }
}

} // namespace pplanner

