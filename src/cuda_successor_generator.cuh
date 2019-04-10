#ifndef CUDA_SUCCESSOR_GNERATOR_H_
#define CUDA_SUCCESSOR_GNERATOR_H_

#include <memory>

#include <curand_kernel.h>

#include "cuda_sas_plus.cuh"
#include "successor_generator.h"

namespace pplanner {

struct CudaSuccessorGenerator {
  int *to_child;
  int *to_data;
  int *data;
  int *no_preconditions_;
  int no_preconditions_size_;
};

extern __constant__ CudaSuccessorGenerator cuda_generator;

__device__
int Count(const CudaSuccessorGenerator &generator, const CudaSASPlus &problem,
          const int *state);

__device__
void Generate(const CudaSuccessorGenerator &generator,
              const CudaSASPlus &problem, const int *state, int *result);

__device__
int Sample(const CudaSuccessorGenerator &generator, const CudaSASPlus &problem,
           const int *state, curandState_t *rng);

std::size_t InitCudaSuccessorGenerator(
    std::shared_ptr<const SuccessorGenerator> generator,
    CudaSuccessorGenerator *cuda_generator);

void FreeCudaSuccessorGenerator(CudaSuccessorGenerator *cuda_generator);

} // namespace pplanner

#endif // CUDA_SUCCESSOR_GNERATOR_H_
