#ifndef CUDA_LANDMARK_COUNT_BASE_H_
#define CUDA_LANDMARK_COUNT_BASE_H_

#include <cstdint>

#include "cuda_sas_plus.cuh"
#include "cuda_landmark/cuda_landmark_graph.cuh"

namespace pplanner {

__device__
int Evaluate(const CudaLandmarkGraph &graph, const CudaSASPlus &problem,
             const int *state, const uint8_t *parent_accepted,
             uint8_t *accepted, uint8_t *status);

} // namespace pplanner

#endif // CUDA_LANDMARK_COUNT_BASE_H_

