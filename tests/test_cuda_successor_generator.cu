#include "cuda_successor_generator.cuh"

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <curand_kernel.h>

#include "cuda_common/cuda_check.cuh"
#include "cuda_common/cuda_random.cuh"
#include "cuda_sas_plus.cuh"
#include "sas_plus.h"
#include "successor_generator.h"

using namespace pplanner;

std::queue<std::string> ExampleSASPlusLines();

__global__
void CountKernel(const CudaSuccessorGenerator generator,
                 const CudaSASPlus problem, const int *state, int *result) {
  *result = Count(generator, problem, state);
}

__global__
void GenerateKernel(const CudaSuccessorGenerator generator,
                    const CudaSASPlus problem, const int *state, int *result) {
  Generate(generator, problem, state, result);
}

void GenerateTest(const CudaSuccessorGenerator &generator,
                  const CudaSASPlus &problem) {
  int *cuda_count = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_count, sizeof(int)));

  std::vector<int> state{0, 1, 0};
  int *cuda_state = nullptr;
  CudaMallocAndCopy((void**)&cuda_state, state.data(), 3 * sizeof(int));

  CountKernel<<<1, 1>>>(generator, problem, cuda_state, cuda_count);
  int count = 0;
  CUDA_CHECK(cudaMemcpy(&count, cuda_count, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(2 == count);

  int *cuda_result = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_result, count * sizeof(int)));
  GenerateKernel<<<1, 1>>>(generator, problem, cuda_state, cuda_result);
  std::vector<int> result(count);
  CUDA_CHECK(cudaMemcpy(result.data(), cuda_result, count * sizeof(int),
                        cudaMemcpyDeviceToHost));
  std::sort(result.begin(), result.end());
  assert(2 == result[0]);
  assert(4 == result[1]);

  state[1] = 0;
  state[2] = 2;
  CUDA_CHECK(cudaMemcpy(cuda_state, state.data(), 3 * sizeof(int),
                        cudaMemcpyHostToDevice));

  CountKernel<<<1, 1>>>(generator, problem, cuda_state, cuda_count);
  CUDA_CHECK(cudaMemcpy(&count, cuda_count, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(2 == count);

  GenerateKernel<<<1, 1>>>(generator, problem, cuda_state, cuda_result);
  CUDA_CHECK(cudaMemcpy(result.data(), cuda_result, count * sizeof(int),
                        cudaMemcpyDeviceToHost));
  std::sort(result.begin(), result.end());
  assert(0 == result[0]);
  assert(2 == result[1]);

  state[0] = 1;
  CUDA_CHECK(cudaMemcpy(cuda_state, state.data(), 3 * sizeof(int),
                        cudaMemcpyHostToDevice));

  CountKernel<<<1, 1>>>(generator, problem, cuda_state, cuda_count);
  CUDA_CHECK(cudaMemcpy(&count, cuda_count, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(2 == count);

  GenerateKernel<<<1, 1>>>(generator, problem, cuda_state, cuda_result);
  CUDA_CHECK(cudaMemcpy(result.data(), cuda_result, count * sizeof(int),
                        cudaMemcpyDeviceToHost));
  std::sort(result.begin(), result.end());
  assert(1 == result[0]);
  assert(3 == result[1]);

  CUDA_CHECK(cudaFree(cuda_count));
  CUDA_CHECK(cudaFree(cuda_result));
  CUDA_CHECK(cudaFree(cuda_state));

  std::cout << "passed GenerateTest" << std::endl;
}

__global__
void SampleKernel(const CudaSuccessorGenerator generator,
                  const CudaSASPlus problem, const int *state, int *result,
                  curandState_t *rng) {
  *result = Sample(generator, problem, state, rng);
}

void SampleTest(const CudaSuccessorGenerator &generator,
                const CudaSASPlus &problem) {
  int *cuda_count = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_count, sizeof(int)));

  std::vector<int> state{0, 1, 0};
  int *cuda_state = nullptr;
  CudaMallocAndCopy((void**)&cuda_state, state.data(), 3 * sizeof(int));

  curandState_t *rng = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&rng, sizeof(curandState_t)));
  SetupStates<<<1, 1>>>(123, rng);

  int *cuda_result = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_result, sizeof(int)));
  SampleKernel<<<1, 1>>>(generator, problem, cuda_state, cuda_result, rng);
  int result;
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(2 == result || 4 == result);

  state[1] = 0;
  state[2] = 2;
  CUDA_CHECK(cudaMemcpy(cuda_state, state.data(), 3 * sizeof(int),
                        cudaMemcpyHostToDevice));

  SampleKernel<<<1, 1>>>(generator, problem, cuda_state, cuda_result, rng);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(0 == result || 2 == result);

  state[0] = 1;
  CUDA_CHECK(cudaMemcpy(cuda_state, state.data(), 3 * sizeof(int),
                        cudaMemcpyHostToDevice));

  SampleKernel<<<1, 1>>>(generator, problem, cuda_state, cuda_result, rng);
  CUDA_CHECK(cudaMemcpy(&result, cuda_result, sizeof(int),
                        cudaMemcpyDeviceToHost));
  assert(1 == result || 3 == result);

  CUDA_CHECK(cudaFree(cuda_count));
  CUDA_CHECK(cudaFree(rng));
  CUDA_CHECK(cudaFree(cuda_result));
  CUDA_CHECK(cudaFree(cuda_state));

  std::cout << "passed SampleTest" << std::endl;
}

int main() {
  auto lines = ExampleSASPlusLines();
  auto problem = std::make_shared<SASPlus>();
  problem->InitFromLines(lines);
  auto generator = std::make_shared<SuccessorGenerator>(problem);

  CudaSASPlus cuda_problem;
  InitCudaSASPlus(problem, &cuda_problem);
  CudaSuccessorGenerator cuda_generator;
  InitCudaSuccessorGenerator(generator, &cuda_generator);

  GenerateTest(cuda_generator, cuda_problem);
  SampleTest(cuda_generator, cuda_problem);

  FreeCudaSuccessorGenerator(&cuda_generator);
  FreeCudaSASPlus(&cuda_problem);
}

std::queue<std::string> ExampleSASPlusLines() {
  std::queue<std::string> q;

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push("0");
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
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("drop ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 -1 1");
  q.push("0 1 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move rooma roomb");
  q.push("0");
  q.push("1");
  q.push("0 0 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move roomb rooma");
  q.push("0");
  q.push("1");
  q.push("0 0 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 0 2");
  q.push("0 1 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 1 2");
  q.push("0 1 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("0");

  return q;
}
