#ifndef CUDA_ZOBRIST_HASH_H_
#define CUDA_ZOBRIST_HASH_H_

#include <cstdint>

#include <memory>
#include <vector>

#include "sas_plus.h"
#include "cuda_sas_plus.cuh"
#include "hash/zobrist_hash.h"

namespace pplanner {

struct CudaZobristHash {
  uint32_t *array;
};

__device__
uint32_t Hash(const CudaZobristHash &hash, const CudaSASPlus &problem,
              const int *state);

__device__
uint32_t HashByDifference(const CudaZobristHash &hash,
                          const CudaSASPlus &problem, int action, uint32_t seed,
                          const int *parent, const int *state);

std::size_t InitCudaZobristHash(std::shared_ptr<const ZobristHash> hash,
                                CudaZobristHash *cuda_hash);

void FreeCudaZobristHash(CudaZobristHash *cuda_hash);

} // namespace pplanner

#endif // CUDA_ZOBRIST_HASH_H_
