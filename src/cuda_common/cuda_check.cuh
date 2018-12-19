#ifndef CUDA_CHECK_H_
#define CUDA_CHECK_H_

#include <cstdio>

#include <cuda_runtime.h>

#define CUDA_CHECK(call)                                                      \
{                                                                             \
  const cudaError_t error = call;                                             \
  if (error != cudaSuccess) {                                                 \
    printf("Error: %s:%d, ", __FILE__, __LINE__);                             \
    printf("code:%d, reason: %s\n", error, cudaGetErrorString(error));        \
    exit(1);                                                                  \
  }                                                                           \
}

inline void CudaMallocAndCopy(void **cuda_array, const void *array,
                              size_t size) {
  CUDA_CHECK(cudaMalloc(cuda_array, size));
  CUDA_CHECK(cudaMemcpy(*cuda_array, array, size, cudaMemcpyHostToDevice));
}

#endif // CUDA_CHECK_H_
