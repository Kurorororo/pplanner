#include "cuda_hash/cuda_zobrist_hash.cuh"

#include <random>

#include "cuda_common/cuda_check.cuh"

namespace pplanner {

using std::vector;

__device__
uint32_t Hash(const CudaZobristHash &hash, const CudaSASPlus &problem,
              const int *state) {
  uint32_t seed = 0;

  for (int i=0, n=problem.n_variables; i<n; ++i)
    seed ^= hash.array[Fact(problem, i, state[i])];

  return seed;
}

__device__
uint32_t HashByDifference(const CudaZobristHash &hash,
                          const CudaSASPlus &problem, int action, uint32_t seed,
                          const int *parent, const int *state) {
  int b = problem.effect_offsets[action];
  int e = problem.effect_offsets[action + 1];

  for (int i = b; i < e; ++i) {
    int var = problem.effect_vars[i];
    seed = seed ^ hash.array[Fact(problem, var, parent[var])]
                ^ hash.array[Fact(problem, var, state[var])];
  }

  return seed;
}

std::size_t InitCudaZobristHash(std::shared_ptr<const ZobristHash> hash,
                                CudaZobristHash *cuda_hash) {
  CudaMallocAndCopy((void**)&cuda_hash->array, hash->array(),
                    hash->array_size() * sizeof(uint32_t));

  return hash->array_size() * sizeof(uint32_t);
}

void FreeCudaZobristHash(CudaZobristHash *cuda_hash) {
  CUDA_CHECK(cudaFree(cuda_hash->array));
}

} // namespace pplanner
