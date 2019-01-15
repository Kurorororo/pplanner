#include "cuda_prefix_sum.cuh"

#include <cstdio>

namespace pplanner {

__device__
void DevicePrefixSum(int thid, int *idata, int *odata, int *tmp, int n) {
  int offset = 1;

  int ai = thid;
  int bi = thid + (n / 2);

  int bank_offset_a = ConflictFreeOffset(ai);
  int bank_offset_b = ConflictFreeOffset(bi);

  tmp[ai + bank_offset_a] = idata[ai];
  tmp[bi + bank_offset_b] = idata[bi];

  for (int d = n >> 1; d > 0; d >>= 1) {
    __syncthreads();

    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      ai += ConflictFreeOffset(ai);
      bi += ConflictFreeOffset(bi);

      tmp[bi] += tmp[ai];
    }

    offset *= 2;
  }

  if (thid == 0) tmp[n - 1 + ConflictFreeOffset(n - 1)] = 0;

  for (int d = 1; d < n; d *= 2) {
    offset >>= 1;
    __syncthreads();

    if (thid < d) {
      int ai = offset * (2 * thid + 1) - 1;
      int bi = offset * (2 * thid + 2) - 1;
      ai += ConflictFreeOffset(ai);
      bi += ConflictFreeOffset(bi);

      int t = tmp[ai];
      tmp[ai] = tmp[bi];
      tmp[bi] += t;
    }
  }

  __syncthreads();

  odata[ai] = tmp[ai + bank_offset_a];
  odata[bi] = tmp[bi + bank_offset_b];
}

__global__ void PrefixSum(int *idata, int *odata, int n) {
  extern __shared__ int tmp[];

  int block_offset = blockIdx.x * blockDim.x;

  DevicePrefixSum(threadIdx.x, &idata[block_offset], &odata[block_offset], tmp,
                  blockDim.x);
}

__global__ void AddBlockPrefixSum(int *idata, int *odata, int n) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;
  if (id < n) odata[threadIdx.x] += idata[blockIdx.x];
}

} // namespace pplanner
