#include "cuda_common/cuda_prefix_sum.cuh"

#include <chrono>
#include <iostream>

#include "cuda_common/cuda_check.cuh"

using namespace pplanner;

void PrefixSumTime() {
  int m = 20;
  int n = 256;

  int *cpu_array = new int[n * m];

  for (int i = 0; i < n * m; ++i)
    cpu_array[i] = rand() % 1000;

  int *cpu_sum = new int[n * m];
  cpu_sum[0] = 0;

  auto chrono_start = std::chrono::system_clock::now();

  for (int i = 1; i < n * m; ++i)
    cpu_sum[i] += cpu_sum[i - 1] + cpu_array[i - 1];

  auto chrono_end = std::chrono::system_clock::now();

  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
     chrono_end - chrono_start).count();
  std::cout << "cpu: " << ns << " ns" << std::endl;

  int *gpu_array = nullptr;
  CudaMallocAndCopy((void**)&gpu_array, cpu_array, n * m * sizeof(int));
  int *gpu_sum = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&gpu_sum, n * m * sizeof(int)));
  int *gpu_offsets = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&gpu_offsets, n * sizeof(int)));
  int *tmp_sum = new int[n * m];
  size_t n_shared = (n + ConflictFreeOffset(n - 1)) * sizeof(int);

  chrono_start = std::chrono::system_clock::now();

  PrefixSum<<<m, n / 2, n_shared>>>(gpu_array, gpu_sum, n * m);
  CUDA_CHECK(cudaMemcpy(tmp_sum, gpu_sum, n * m * sizeof(int),
                        cudaMemcpyDeviceToHost));

  int small[m];
  small[0] = 0;

  for (int i = 1; i < m; ++i)
    small[i] = tmp_sum[i * n - 1];

  CUDA_CHECK(cudaMemcpy(gpu_offsets, small, n * sizeof(int),
                        cudaMemcpyHostToDevice));
  AddBlockPrefixSum<<<m, n>>>(gpu_offsets, gpu_sum, n * m);
  CUDA_CHECK(cudaMemcpy(tmp_sum, gpu_sum, n * m * sizeof(int),
                        cudaMemcpyDeviceToHost));

  chrono_end = std::chrono::system_clock::now();
  ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
     chrono_end - chrono_start).count();
  std::cout << "gpu: " << ns << " ns" << std::endl;

  delete[] cpu_array;
  delete[] cpu_sum;
  delete[] tmp_sum;
  CUDA_CHECK(cudaFree(gpu_array));
  CUDA_CHECK(cudaFree(gpu_sum));
}

int main() {
  PrefixSumTime();

  return 0;
}
