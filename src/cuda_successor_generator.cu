#include "cuda_successor_generator.cuh"

#include "cuda_common/cuda_check.cuh"

namespace pplanner {

__constant__ CudaSuccessorGenerator cuda_generator;

__device__
void DFSample(const CudaSuccessorGenerator &generator,
              const CudaSASPlus &problem, const int *state, int index,
              int current, unsigned int &k, int &result, curandState_t *rng) {
  int offset = index - problem.var_offsets[current];

  for (int i = current, n = problem.n_variables; i < n; ++i) {
    int next = Fact(problem, i, state[i]) + offset;
    int b = generator.to_data[next];
    int e = generator.to_data[next + 1];

    for (int j = b; j < e; ++j) {
      if (curand(rng) % k == 0) result = generator.data[j];
      ++k;
    }

    int child = generator.to_child[next];
    if (child == -1) continue;

    DFSample(generator, problem, state, child, i + 1, k, result, rng);
  }
}

__device__
int Sample(const CudaSuccessorGenerator &generator, const CudaSASPlus &problem,
           const int *state, curandState_t *rng) {
  int result = -1;
  unsigned int k = 1;
  DFSample(generator, problem, state, 0, 0, k, result, rng);

  return result;
}

__device__
void DFSCount(const CudaSuccessorGenerator &generator,
              const CudaSASPlus &problem, const int *state, int index,
              int current, int &count) {
  int offset = index - problem.var_offsets[current];

  for (int i = current, n = problem.n_variables; i < n; ++i) {
    int next = Fact(problem, i, state[i]) + offset;
    count += generator.to_data[next + 1] - generator.to_data[next];
    int child = generator.to_child[next];
    if (child == -1) continue;

    DFSCount(generator, problem, state, child, i + 1, count);
  }
}

__device__
int Count(const CudaSuccessorGenerator &generator, const CudaSASPlus &problem,
          const int *state) {
  int count = 0;
  DFSCount(generator, problem, state, 0, 0, count);

  return count;
}

__device__
void DFS(const CudaSuccessorGenerator &generator,
         const CudaSASPlus &problem, const int *state, int index, int current,
         int &count, int *result) {
  int offset = index - problem.var_offsets[current];

  for (int i = current, n = problem.n_variables; i < n; ++i) {
    int next = Fact(problem, i, state[i]) + offset;
    int b = generator.to_data[next];
    int e = generator.to_data[next + 1];

    for (int j = b; j < e; ++j)
      result[count++] = generator.data[j];

    int child = generator.to_child[next];
    if (child == -1) continue;

    DFS(generator, problem, state, child, i + 1, count, result);
  }
}

__device__
void Generate(const CudaSuccessorGenerator &generator,
              const CudaSASPlus &problem, const int *state, int *result) {
  int count = 0;
  DFS(generator, problem, state, 0, 0, count, result);
}

std::size_t InitCudaSuccessorGenerator(
    std::shared_ptr<const SuccessorGenerator> generator,
    CudaSuccessorGenerator *cuda_generator) {
  CudaMallocAndCopy((void**)&cuda_generator->to_child,
                    generator->to_child_data(),
                    generator->to_child_size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_generator->to_data, generator->to_data(),
                    generator->to_data_size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_generator->data, generator->data(),
                    generator->data_size() * sizeof(int));

  return (generator->to_child_size() + generator->to_data_size()
          + generator->data_size()) * sizeof(int);
}

void FreeCudaSuccessorGenerator(CudaSuccessorGenerator *cuda_generator) {
  CUDA_CHECK(cudaFree(cuda_generator->to_child));
  CUDA_CHECK(cudaFree(cuda_generator->to_data));
  CUDA_CHECK(cudaFree(cuda_generator->data));
}

} // namespace pplanner
