#include "trie/gpu_trie.h"

#include <cstdio>

#include "common/cuda_check.h"
#include "common/malloc_check.h"

using std::vector;

namespace rwls {

__device__
void RecursiveFind(const TrieTable &table, const DeviceDomain &domain,
                   const int* variables, int index, size_t current,
                   GVector<int> &result) {
  int prefix = index - domain.fact_offset[current];
  for (size_t i=current, n=domain.variables_size; i<n; ++i) {
    int next = domain.fact_offset[i] + variables[i] + prefix;
    int offset = table.to_data[next];
    if (offset != -1) {
      size_t size = table.data_size_array[offset];
      for (size_t j=0; j<size; ++j) {
        result.push_back(table.data[offset][j]);
      }
    }
    if (table.to_child[next] == -1) continue;
    RecursiveFind(table, domain, variables, table.to_child[next], i+1, result);
  }
}

__device__
void FindFromTable(const TrieTable &table, const DeviceDomain &domain,
                   const int *variables, GVector<int> &result) {
  result.resize(0);
  RecursiveFind(table, domain, variables, 0, 0, result);
}

__device__
void RecursiveFind(const TrieTable &table, const DeviceDomain &domain,
                   const int *variables, int index, size_t current,
                   int *k, int *result) {
  int prefix = index - domain.fact_offset[current];

  for (size_t i=current, n=domain.variables_size; i<n; ++i) {
    int next = domain.fact_offset[i] + variables[i] + prefix;
    int offset = table.to_data[next];

    if (offset != -1) {
      size_t size = table.data_size_array[offset];

      for (size_t j=0; j<size; ++j)
        result[(*k)++] = table.data[offset][j];
    }

    int child = table.to_child[next];

    if (child == -1) continue;

    RecursiveFind(table, domain, variables, child, i+1, k, result);
  }
}

__device__
void FindFromTable(const TrieTable &table, const DeviceDomain &domain,
                   const int *variables, int *result) {
  int k = 0;
  RecursiveFind(table, domain, variables, 0, 0, &k, result);
}

__device__
void RecursiveSample(const TrieTable &table, const DeviceDomain &domain,
                     const int *variables, int index, size_t current,
                     unsigned int *k, curandState_t *state, int *result) {
  int prefix = index - domain.fact_offset[current];
  for (size_t i=current, n=domain.variables_size; i<n; ++i) {
    int next = domain.fact_offset[i] + variables[i] + prefix;
    int offset = table.to_data[next];
    if (offset != -1) {
      size_t size = table.data_size_array[offset];
      for (size_t j=0; j<size; ++j) {
        if (curand(state) % *k == 0)
          *result = table.data[offset][j];
        ++(*k);
      }
    }
    int child = table.to_child[next];
    if (child == -1) continue;
    RecursiveSample(table, domain, variables, child, i+1, k, state, result);
  }
}

__device__
int SampleFromTable(const TrieTable &table, const DeviceDomain &domain,
                    const int *variables, curandState_t *state) {
  int result = -1;
  unsigned int k = 1;
  RecursiveSample(table, domain, variables, 0, 0, &k, state, &result);
  return result;
}

__device__
void RecursiveCount(const TrieTable &table, const DeviceDomain &domain,
                    const int *variables, int index, size_t current,
                    int *result) {
  int prefix = index - domain.fact_offset[current];

  for (size_t i=current, n=domain.variables_size; i<n; ++i) {
    int next = domain.fact_offset[i] + variables[i] + prefix;
    int offset = table.to_data[next];

    if (offset != -1) {
      int size = table.data_size_array[offset];
      *result += size;
    }

    int child = table.to_child[next];
    if (child == -1) continue;
    RecursiveCount(table, domain, variables, child, i+1, result);
  }
}

__device__
int CountFromTable(const TrieTable &table, const DeviceDomain &domain,
                   const int *variables) {
  int result = 0;
  RecursiveCount(table, domain, variables, 0, 0, &result);

  return result;
}

void CudaInitializeTable(const TrieTable &h_table, TrieTable *d_table) {
  size_t size = h_table.size;
  size_t data_size = h_table.data_size;
  d_table->size = size;
  d_table->data_size = data_size;
  CUDA_CHECK(cudaMalloc((void**)&d_table->to_child, size*sizeof(int)));
  CUDA_CHECK(cudaMemcpy(d_table->to_child, h_table.to_child,
                        size*sizeof(int), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMalloc((void**)&d_table->to_data, size*sizeof(int)));
  CUDA_CHECK(cudaMemcpy(d_table->to_data, h_table.to_data, size*sizeof(int),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMalloc((void**)&d_table->data_size_array,
                        data_size*sizeof(size_t)));
  CUDA_CHECK(cudaMemcpy(d_table->data_size_array, h_table.data_size_array,
                        data_size*sizeof(size_t), cudaMemcpyHostToDevice));
  int **tmp = (int**)malloc(data_size*sizeof(int*));
  ALLOC_CHECK(tmp);
  for (size_t i=0; i<data_size; ++i) {
    CUDA_CHECK(cudaMalloc((void**)&tmp[i],
                          h_table.data_size_array[i]*sizeof(int)));
    CUDA_CHECK(cudaMemcpy(tmp[i], h_table.data[i],
                          h_table.data_size_array[i]*sizeof(int),
                          cudaMemcpyHostToDevice));
  }
  CUDA_CHECK(cudaMalloc((void**)&d_table->data, data_size*sizeof(int*)));
  CUDA_CHECK(cudaMemcpy(d_table->data, tmp, data_size*sizeof(int*),
                        cudaMemcpyHostToDevice));
  free(tmp);
}

void CudaFinalizeTable(TrieTable *d_table) {
  CUDA_CHECK(cudaFree(d_table->to_child));
  CUDA_CHECK(cudaFree(d_table->to_data));
  CUDA_CHECK(cudaFree(d_table->data_size_array));

  size_t data_size = d_table->data_size;
  int **tmp = (int**)malloc(data_size*sizeof(int*));
  ALLOC_CHECK(tmp);
  CUDA_CHECK(cudaMemcpy(tmp, d_table->data, data_size*sizeof(int*),
                        cudaMemcpyDeviceToHost));
  for (size_t i=0; i<data_size; ++i) {
    CUDA_CHECK(cudaFree(tmp[i]));
  }
  CUDA_CHECK(cudaFree(d_table->data));
  free(tmp);
}

__device__
void DevicePrintTable(const TrieTable &table) {
  for (size_t i=0; i<table.size; ++i)
    printf("%d ", table.to_child[i]);
  printf("\n");
  for (size_t i=0; i<table.size; ++i)
    printf("%d ", table.to_data[i]);
  printf("\n");
  for (size_t i=0; i<table.data_size; ++i) {
    for (int j=0; j<table.data_size_array[i]; ++j)
      printf("%d ", table.data[i][j]);
  }
  printf("\n");
}

} // namespace rwls
