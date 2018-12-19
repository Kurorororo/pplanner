#include "cuda_sas_plus.cuh"

#include <iostream>

#include "cuda_common/cuda_check.cuh"

#include <cassert>

using namespace pplanner;

std::queue<std::string> ExampleSASPlusLines(bool unit_cost=true);

__global__
void IsGoalKernel(const CudaSASPlus problem, const int *state, bool *result) {
  *result = IsGoal(problem, state);
}

__global__
void ApplyEffectKernel(const CudaSASPlus problem, int i, const int *state,
                       int *child) {
  ApplyEffect(problem, i, state, child);
}

int main() {
  bool *cuda_result = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_result, sizeof(bool)));

  auto lines = ExampleSASPlusLines();
  std::shared_ptr<SASPlus> problem = std::make_shared<SASPlus>();
  problem->InitFromLines(lines);

  CudaSASPlus cuda_problem;
  InitCudaSASPlus(problem, &cuda_problem);

  int *cuda_state = nullptr;
  CudaMallocAndCopy((void**)&cuda_state, problem->initial().data(),
                    problem->n_variables() * sizeof(int));

  int *cuda_child = nullptr;
  CudaMallocAndCopy((void**)&cuda_child, problem->initial().data(),
                    problem->n_variables() * sizeof(int));

  IsGoalKernel<<<1, 1>>>(cuda_problem, cuda_child, cuda_result);
  bool result;
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(bool),
                        cudaMemcpyDeviceToHost));
  assert(!result);

  auto modified = problem->initial();
  modified[2] = 1;
  CUDA_CHECK(cudaMemcpy(cuda_child, modified.data(),
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyHostToDevice));
  IsGoalKernel<<<1, 1>>>(cuda_problem, cuda_child, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(bool),
                        cudaMemcpyDeviceToHost));
  assert(result);
  std::cout << "passed IsGoal" << std::endl;

  ApplyEffectKernel<<<1, 1>>>(cuda_problem, 4, cuda_state, cuda_child);
  modified = problem->initial();
  problem->ApplyEffect(4, modified);
  auto child = problem->initial();
  CUDA_CHECK(cudaMemcpy(child.data(), cuda_child,
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(child == modified);

  ApplyEffectKernel<<<1, 1>>>(cuda_problem, 2, cuda_child, cuda_state);
  problem->ApplyEffect(2, modified);
  CUDA_CHECK(cudaMemcpy(child.data(), cuda_state,
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(child == modified);

  ApplyEffectKernel<<<1, 1>>>(cuda_problem, 1, cuda_state, cuda_child);
  problem->ApplyEffect(1, modified);
  CUDA_CHECK(cudaMemcpy(child.data(), cuda_child,
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(child == modified);
  std::cout << "passed ApplyEffect" << std::endl;

  CUDA_CHECK(cudaFree(cuda_result));
  CUDA_CHECK(cudaFree(cuda_state));
  CUDA_CHECK(cudaFree(cuda_child));
  FreeCudaSASPlus(&cuda_problem);

  return 0;
}

std::queue<std::string> ExampleSASPlusLines(bool unit_cost) {
  std::queue<std::string> q;

  std::string cost = "10";
  std::string metric = unit_cost ? "0" : "1";

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push(metric);
  q.push("end_metric");
  q.push("3");
  q.push("begin_variable");
  q.push("var0");
  q.push("-1");
  q.push("2");
  q.push("Atom at-robby(rooma)");
  q.push("Atom at-robby(roomb)");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var1");
  q.push("-1");
  q.push("2");
  q.push("Atom carry(ball1, left)");
  q.push("Atom free(left)");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var2");
  q.push("-1");
  q.push("3");
  q.push("Atom at(ball1, rooma)");
  q.push("Atom at(ball1, roomb)");
  q.push("<none of those>");
  q.push("end_variable");
  q.push("1");
  q.push("begin_mutex_group");
  q.push("3");
  q.push("2 0");
  q.push("2 1");
  q.push("1 0");
  q.push("end_mutex_group");
  q.push("begin_state");
  q.push("0");
  q.push("1");
  q.push("0");
  q.push("end_state");
  q.push("begin_goal");
  q.push("1");
  q.push("2 1");
  q.push("end_goal");
  q.push("6");
  q.push("begin_operator");
  q.push("drop ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 -1 0");
  q.push("0 1 0 1");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("drop ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 -1 1");
  q.push("0 1 0 1");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move rooma roomb");
  q.push("0");
  q.push("1");
  q.push("0 0 0 1");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move roomb rooma");
  q.push("0");
  q.push("1");
  q.push("0 0 1 0");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 0 2");
  q.push("0 1 1 0");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 1 2");
  q.push("0 1 1 0");
  q.push(cost);
  q.push("end_operator");
  q.push("0");

  return q;
}
