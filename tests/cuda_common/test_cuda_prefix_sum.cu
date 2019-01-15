#include "cuda_common/cuda_prefix_sum.cuh"

#include <cassert>

#include <iostream>

#include "cuda_common/cuda_check.cuh"

using namespace pplanner;

void PrefixSumTest() {
  int n = 2048;

  int *cpu_array = new int[n];

  for (int i = 0; i < n; ++i)
    cpu_array[i] = rand() % 1000;

  int *cpu_sum = new int[n];
  cpu_sum[0] = 0;

  for (int i = 1; i < n; ++i)
    cpu_sum[i] += cpu_sum[i - 1] + cpu_array[i - 1];

  int *gpu_array = nullptr;
  CudaMallocAndCopy((void**)&gpu_array, cpu_array, n * sizeof(int));
  int *gpu_sum = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&gpu_sum, n * sizeof(int)));
  int *tmp_sum = new int[n];
  size_t n_shared = (n + ConflictFreeOffset(n - 1)) * sizeof(int);
  PrefixSum<<<1, n / 2, n_shared>>>(gpu_array, gpu_sum, n);
  CUDA_CHECK(cudaMemcpy(tmp_sum, gpu_sum, n * sizeof(int),
                        cudaMemcpyDeviceToHost));

  for (int i = 0; i < n; ++i)
    assert(cpu_sum[i] == tmp_sum[i]);

  delete[] cpu_array;
  delete[] cpu_sum;
  delete[] tmp_sum;
  CUDA_CHECK(cudaFree(gpu_array));
  CUDA_CHECK(cudaFree(gpu_sum));

  std::cout << "passed PrefixSumTest" << std::endl;
}

int main() {
  PrefixSumTest();

  return 0;
}
