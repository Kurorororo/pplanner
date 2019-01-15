#include "cuda_search_graph.cuh"

#include "cuda_hash/cuda_zobrist_hash.cuh"

namespace pplanner {

__device__
void Pack(const CudaSearchGraph &graph, const int *state, uint32_t *packed) {
  int index = 0;

  for (int i = 0, n = graph.block_size; i < n; ++i) {
    uint32_t tmp = 0;

    for (int j=0, m=graph.var_per_block[i]; j<m; ++j) {
      tmp |= static_cast<uint32_t>(state[index]) << graph.shift[index];
      ++index;
    }

    packed[i] = tmp;
  }
}

__device__
void UnPack(const CudaSearchGraph &graph, const uint32_t *packed, int *state) {
  for (int i = 0, n = graph.n_variables; i < n; ++i) {
    int index = graph.block_index[i];
    state[i] = static_cast<int>(
        (packed[index] & graph.mask[i]) >> graph.shift[i]);
  }
}

__device__
void GenerateNode(int node, int action, int parent, uint32_t hash_value,
                  uint32_t d_hash_value, const uint32_t *packed,
                  CudaSearchGraph *graph) {
  graphs->actions[node] = action;
  graphs->parents[node] = parent;
  graphs->hash_values[node] = hash_values;
  graphs->d_hash_values[node] = d_hash_values;
  std::size_t index = static_cast<std::size_t>(node) * graph->block_size;
  memcpy(&graph->states[index], packed);
}

__device__
void SetLandmark(int node, int n_bytes, const uint8_t *landmark,
                 CudaSearchGraph *graph) {
  std::size_t index = static_cast<std::size_t>(node)
    * static_cast<std::size_t>(n_bytes);
  memcpy(&graph.states[index], landmark, n_bytes * sizeof(uint8_t));
}

__device__
void State(const CudaSearchGraph &graph, int node, int *state) {
  std::size_t b_size = graph.block_size;
  uint32_t *packed = graph.states + static_cast<std::size_t>(node) * b_size;
  Unpack(graph, packed, state);
}

__device__
void BytesEqual(size_t size, const uint32_t *a, const uint32_t *b) {
  for (int i = 0; i < size; ++i)
    if (a[i] != b[i]) return false;

  return true;
}

__device__
int GetClosed(const CudaSearchGraph &graph, const int *closed, int node) {
  uint32_t i = graph.hash_values[node] & graph.closed_mask;

  if (closed[i] == -1) return -1;

  std::size_t b_size = graph.block_size;
  uint32_t *found = graph.states + static_cast<std::size_t>(closed[i]) * b_size;
  uint32_t *packed = graph.states + static_cast<std::size_t>(i) * b_size;

  if (BytesEqual(b_size, packed, found)) return closed[i];

  return -1;
}

__device__
int GetClosed(const CudaSearchGraph &graph, const int *closed, uint32_t hash,
              const uint32_t *packed) {
  uint32_t i = hash & graph.closed_mask;

  if (closed[i] == -1) return -1;

  std::size_t b_size = graph.block_size;
  uint32_t *found = graph.states + static_cast<std::size_t>(closed[i]) * b_size;

  if (BytesEqual(b_size, packed, found)) return closed[i];

  return -1;
}

__device__
void Close(const CudaSearchGraph &graph, int node, int *closed) {
  uint32_t i = graph.hash_values[node] & graph.closed_mask;
  closed[i] = node;
}

__device__
void Push(int h, int node, CudaSearchGraph *graph, int *next_list,
          int *prev_list) {
  int last = prev_list[h];
  graph.prev[node] = last
  prev_list[h] = node;
  graph.next[node] = -1;

  if (last == -1)
    next_list[h] = node;
  else
    graph.next[last] = node;
}

__device__
int Pop(int h_max, int *h_min, CudaSearchGraph *graph, int *next_list,
        int *prev_list) {
  for (int i = *h_min; i < h_max; ++i) {
    int node = next_list[i];

    if (node == -1) continue;

    int new_next = graph.next[node];
    next_list[i] = new_next;

    if (new_next == -1) prev_list[i] = -1;

    *h_min = i;

    return node;
  }

  *h_min = h_max;

  return -1;
}

void InitCudaSearchGraph(std::shared_ptr<const SASPlus> problem,
                         int closed_exponent, int gpu_ram,
                         std::size_t n_landmark_bytes, CudaSearchGraph *graph) {
  graph->n_variables = problem->n_variables();
  graph->n_landmark_bytes = n_landmark_bytes;
  graph->closed_mask = (1u << graph.closed_exponent) - 1;
  gpu_ram -= sizeof(int) + 2 * sizeof(uint32_t);

  auto packer = std::unique_ptr<StatePacker>(new StatePacker(problem));
  graph->block_size = packer->block_size();
  CudaMallocAndCopy((void**)&graph->block_index, packer->block_index(),
                    packer->block_index_size() * sizeof(int));
  CudaMallocAndCopy((void**)&graph->var_per_block, packer->var_per_block(),
                    packer->var_per_block_size() * sizeof(int));
  CudaMallocAndCopy((void**)&graph->shift, packer->shift(),
                    packer->shift_size() * sizeof(int));
  CudaMallocAndCopy((void**)&graph->mask, packer->mask(),
                    packer->mask() * sizeof(uint32_t));
  gpu_ram -= sizeof(std::size_t);
  gpu_ram -= packer->block_index_size() * sizeof(int);
  gpu_ram -= packer->var_per_block_size() * sizeof(int);
  gpu_ram -= packer->shift_size() * sizeof(int);
  gpu_ram -= packer->mask_size() * sizeof(uint32_t);

  gpu_ram -= sizeof(std::size_t);
  std::size_t node_size = 4 * sizeof(int);
  node_size += (packer->block_size() + 2) * sizeof(uint32_t);
  node_size += n_landmark_bytes * sizeof(uint8_t);
  graph->node_max = gpu_ram / node_size;
  CUDA_CHECK(cudaMalloc((void**)&graph->actions,
             graph->node_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&graph->parents,
             graph->node_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&graph->next, graph->node_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&graph->prev, graph->node_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&graph->states,
  CUDA_CHECK(cudaMalloc((void**)&graph->states,
             graph->node_max * packer->block_size() * sizeof(uint32_t)));
  CUDA_CHECK(cudaMalloc((void**)&graph->hash_values,
             graph->node_max * sizeof(uint32_t)));
  CUDA_CHECK(cudaMalloc((void**)&graph->d_hash_values,
             graph->node_max * sizeof(uint32_t)));

  graph->landmarks = nullptr;

  if (n_landmark_bytes > 0)
    CUDA_CHECK(cudaMalloc((void**)&graph->landmarks,
               graph->node_max * n_landmark_bytes * sizeof(uint8_t)));
}

void FreeCudaSearchGraph(CudaSearchGraph *graph) {
  CUDA_CHECK(cudaFree(graph->actions));
  CUDA_CHECK(cudaFree(graph->parents));
  CUDA_CHECK(cudaFree(graph->next));
  CUDA_CHECK(cudaFree(graph->prev));
  CUDA_CHECK(cudaFree(graph->states));
  CUDA_CHECK(cudaFree(graph->hash_values));
  CUDA_CHECK(cudaFree(graph->d_hash_values));

  if (graph->landmarks != nullptr)
    CUDA_CHECK(cudaFree(graph->landmarks));
}

} // namespace pplanner
