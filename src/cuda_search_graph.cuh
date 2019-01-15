#ifndef CUDA_SEARCH_GRAPH_H_
#define CUDA_SEARCH_GRAPH_H_

#include <cstdint>

#include <memory>

#include "cuda_sas_plus.cuh"
#include "sas_plus.cuh"

namespace pplanner {

struct CudaSearchGraph {
  int n_variables;
  std::size_t n_landmark_bytes;
  std::size_t node_max;
  uint32_t closed_mask;
  int *actions;
  int *parents;
  int *next;
  int *prev;
  int *states;
  uint32_t *packed;
  uint32_t *hash_values;
  uint32_t *d_hash_values;
  uint8_t *landmarks;
  // StatePacker
  std::size_t block_size;
  int *block_index;
  int *var_per_block;
  int *shift;
  uint32_t *mask;
};

__device__
void Pack(const CudaSearchGraph &graph, const int *state, uint32_t *packed);

__device__
void UnPack(const CudaSearchGraph &graph, const uint32_t *packed, int *state);

__device__
void GenerateNode(int node, int action, int parent, uint32_t hash_value,
                  uint32_t d_hash_value, const uint32_t *packed,
                  CudaSearchGraph *graph);

__device__
void SetLandmark(int node, int n_bytes, const uint8_t *landmark,
                 CudaSearchGraph *graph);

__device__
void State(const CudaSearchGraph &graph, int node, int *state);

__device__
inline uint8_t* GetLandmark(CudaSearchGraph &graph, int node) {
  std::size_t index = static_cast<std::size_t>(node) * graph.n_landmark_bytes;

  return &graph.landmark[index];
}

__device__
int GetClosed(const CudaSearchGraph &graph, const int *closed, int node);

__device__
int GetClosed(const CudaSearchGraph &graph, const int *closed, uint32_t hash,
              const uint32_t *packed);

__device__
void Close(const CudaSearchGraph &graph, int node, int *closed);

__device__
void Push(int h, int node, CudaSearchGraph *graph, int *next_list,
          int *prev_list);
__device__
int Pop(int h_max, int *h_min, CudaSearchGraph *graph, int *next_list,
        int *prev_list);

void InitCudaSearchGraph(std::shared_ptr<const SASPlus> problem,
                         int n_landmark_bytes, std::size_t gpu_ram,
                         CudaSearchGraph *graph);

void FreeCudaSearchGraph(CudaSearchGraph *graph);

} // namespace pplanner

#endif // CUDA_SEARCH_GRAPH_H_
