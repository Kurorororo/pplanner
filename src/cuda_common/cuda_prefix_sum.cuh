#ifndef CUDA_PREFIX_SUM_H_
#define CUDA_PREFIX_SUM_H_

namespace pplanner {

constexpr int kNBanks = 32;
constexpr int kLogNBanks = 5;

__device__ __host__ __inline__ int ConflictFreeOffset(int n) {
  return n >> kLogNBanks;
}

__device__
void DevicePrefixSum(int thid, int *idata, int *odata, int *tmp, int n);

__global__ void PrefixSum(int *idata, int *odata, int n);

__global__ void AddBlockPrefixSum(int *idata, int *odata, int n);

} // namespace pplanner

#endif // CUDA_PREFIX_SUM_H_
