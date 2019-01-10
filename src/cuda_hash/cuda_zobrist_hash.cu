#include "hash/zobrist_hash.h"

#include <random>

namespace pplanner {

using std::vector;

ZobristHash::ZobristHash(std::shared_ptr<const SASPlus> problem, uint32_t seed)
  : n_(static_cast<int>(problem->n_variables())),
    problem_(problem) {
  std::mt19937 mt(seed);
  array_.resize(problem_->n_facts());

  for (auto &v : array_)
    v = mt();
}

uint32_t Hash(const CudaZobristHash &hash, const CudaSASPlus &problem,
              const int *state) {
  uint32_t seed = 0;

  for (int i=0, n=problem.n_variables; i<n; ++i)
    seed ^= array_[Fact(problem, i, state[i])];

  return seed;
}

uint32_t HashByDifference(const CudaZobristHash &hash,
                          const CudaSASPlus &problem, int action, uint32_t seed,
                          const int *parent, const int *state) {
  int b = problem.effect_offsets[i];
  int e = problem.effect_offsets[i + 1];

  for (int i = b; i < e; ++i)
    seed = seed ^ array_[Fact(problem, i, parent[i])]
                ^ array_[Fact(problem, i, state[i])];
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
