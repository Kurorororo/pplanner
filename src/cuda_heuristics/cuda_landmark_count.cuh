#ifndef CUDA_LANDMARK_COUNT_H_
#define CUDA_LANDMARK_COUNT_H_

namespace pplanner {

int CudaRWLandmarkCount(const CudaSASPlus &problem,
                        const CudaLandmarkGraph &graph);


} // namespace pplanner

#endif // CUDA_LANDMARK_COUNT_H_

