#include "cuda_sas_plus.cuh"

#include <iostream>
#include <memory>

#include "cuda_common/cuda_check.cuh"

#include <cassert>

using namespace pplanner;

std::queue<std::string> ExampleSASPlusLines(bool unit_cost=true);

__global__
void FactKernel(const CudaSASPlus problem, int var, int value, int *result) {
  *result = Fact(problem, var, value);
}

void FactTest(std::shared_ptr<const SASPlus> problem,
              const CudaSASPlus &cuda_problem) {
  int *cuda_result = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_result, sizeof(int)));
  int result = 0;

  FactKernel<<<1, 1>>>(cuda_problem, 0, 0, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(0 == result);

  FactKernel<<<1, 1>>>(cuda_problem, 0, 1, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(1 == result);

  FactKernel<<<1, 1>>>(cuda_problem, 1, 0, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(2 == result);

  FactKernel<<<1, 1>>>(cuda_problem, 1, 1, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(3 == result);

  FactKernel<<<1, 1>>>(cuda_problem, 2, 0, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(4 == result);

  FactKernel<<<1, 1>>>(cuda_problem, 2, 1, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(5 == result);

  FactKernel<<<1, 1>>>(cuda_problem, 2, 2, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(6 == result);

  CUDA_CHECK(cudaFree(cuda_result));

  std::cout << "passed FactTest" << std::endl;
}

__global__
void IsGoalKernel(const CudaSASPlus problem, const int *state, bool *result) {
  *result = IsGoal(problem, state);
}

void IsGoalTest(std::shared_ptr<const SASPlus> problem,
                const CudaSASPlus &cuda_problem) {
  bool *cuda_result = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_result, sizeof(bool)));
  bool result;

  auto state = problem->initial();
  int *cuda_state = nullptr;
  CudaMallocAndCopy((void**)&cuda_state, state.data(),
                    problem->n_variables() * sizeof(int));

  IsGoalKernel<<<1, 1>>>(cuda_problem, cuda_state, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(bool),
                        cudaMemcpyDeviceToHost));
  assert(!result);

  state[2] = 1;
  CUDA_CHECK(cudaMemcpy(cuda_state, state.data(),
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyHostToDevice));
  IsGoalKernel<<<1, 1>>>(cuda_problem, cuda_state, cuda_result);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(bool),
                        cudaMemcpyDeviceToHost));
  assert(result);

  CUDA_CHECK(cudaFree(cuda_result));
  CUDA_CHECK(cudaFree(cuda_state));

  std::cout << "passed IsGoalTest" << std::endl;
}

__global__
void ApplyEffectKernel(const CudaSASPlus problem, int i, const int *state,
                       int *child) {
  ApplyEffect(problem, i, state, child);
}

void ApplyEffectTest(std::shared_ptr<const SASPlus> problem,
                     const CudaSASPlus &cuda_problem) {
  auto state = problem->initial();
  auto child = state;
  int *cuda_state = nullptr;
  CudaMallocAndCopy((void**)&cuda_state, state.data(),
                    problem->n_variables() * sizeof(int));
  int *cuda_child = nullptr;
  CudaMallocAndCopy((void**)&cuda_child, child.data(),
                    problem->n_variables() * sizeof(int));

  ApplyEffectKernel<<<1, 1>>>(cuda_problem, 4, cuda_state, cuda_child);
  problem->ApplyEffect(4, state);
  CUDA_CHECK(cudaMemcpy(child.data(), cuda_child,
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(state == child);

  ApplyEffectKernel<<<1, 1>>>(cuda_problem, 2, cuda_child, cuda_state);
  problem->ApplyEffect(2, state);
  CUDA_CHECK(cudaMemcpy(child.data(), cuda_state,
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(state == child);

  ApplyEffectKernel<<<1, 1>>>(cuda_problem, 1, cuda_state, cuda_child);
  problem->ApplyEffect(1, state);
  CUDA_CHECK(cudaMemcpy(child.data(), cuda_child,
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(state == child);

  state = std::vector<int>{1, 1, 1, 0};
  CUDA_CHECK(cudaMemcpy(cuda_state, state.data(),
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyHostToDevice));
  ApplyEffectKernel<<<1, 1>>>(cuda_problem, 6, cuda_state, cuda_child);
  problem->ApplyEffect(6, state);
  CUDA_CHECK(cudaMemcpy(child.data(), cuda_child,
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(state == child);

  state = std::vector<int>{1, 1, 1, 1};
  CUDA_CHECK(cudaMemcpy(cuda_state, state.data(),
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyHostToDevice));
  ApplyEffectKernel<<<1, 1>>>(cuda_problem, 6, cuda_state, cuda_child);
  problem->ApplyEffect(6, state);
  CUDA_CHECK(cudaMemcpy(child.data(), cuda_child,
                        problem->n_variables() * sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(state == child);

  CUDA_CHECK(cudaFree(cuda_state));
  CUDA_CHECK(cudaFree(cuda_child));

  std::cout << "passed ApplyEffectTest" << std::endl;
}

int main() {
  auto lines = ExampleSASPlusLines();
  std::shared_ptr<SASPlus> problem = std::make_shared<SASPlus>();
  problem->InitFromLines(lines);

  CudaSASPlus cuda_problem;
  InitCudaSASPlus(problem, &cuda_problem);

  FactTest(problem, cuda_problem);
  IsGoalTest(problem, cuda_problem);
  ApplyEffectTest(problem, cuda_problem);

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
  q.push("4");
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
  q.push("begin_variable");
  q.push("var3");
  q.push("-1");
  q.push("2");
  q.push("Atom low");
  q.push("Atom high");
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
  q.push("0");
  q.push("end_state");
  q.push("begin_goal");
  q.push("1");
  q.push("2 1");
  q.push("end_goal");
  q.push("7");
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
  q.push("begin_operator");
  q.push("pick_conditional ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("3");
  q.push("0 2 1 2");
  q.push("0 1 1 0");
  q.push("1 3 1 0 -1 0");
  q.push(cost);
  q.push("end_operator");
  q.push("0");

  return q;
}
