#include "cuda_search_graph.cuh"

#include "cuda_common/cuda_check.cuh"
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
  graph->actions[node] = action;
  graph->parents[node] = parent;
  graph->hash_values[node] = hash_value;
  graph->d_hash_values[node] = d_hash_value;
  graph->closed[node] = -1;
  std::size_t index = static_cast<std::size_t>(node) * graph->block_size;
  memcpy(&graph->states[index], packed, graph->block_size * sizeof(uint32_t));
}

__device__
void SetLandmark(int node, int n_bytes, const uint8_t *landmark,
                 CudaSearchGraph *graph) {
  std::size_t index = static_cast<std::size_t>(node)
    * static_cast<std::size_t>(graph->n_landmarks_bytes);
  memcpy(&graph->landmarks[index], landmark, n_bytes * sizeof(uint8_t));
}

__device__
void State(const CudaSearchGraph &graph, int node, int *state) {
  std::size_t b_size = graph.block_size;
  uint32_t *packed = graph.states + static_cast<std::size_t>(node) * b_size;
  UnPack(graph, packed, state);
}

__device__
bool BytesEqual(size_t size, const uint32_t *a, const uint32_t *b) {
  for (int i = 0; i < size; ++i)
    if (a[i] != b[i]) return false;

  return true;
}

__device__
int GetClosed(const CudaSearchGraph &graph, const int *closed, int node) {
  uint32_t i = graph.hash_values[node] & graph.closed_mask;
  std::size_t b = graph.block_size;
  uint32_t *packed = graph.states + static_cast<std::size_t>(node) * b;
  int current = closed[i];

  while (current != -1) {
    uint32_t *found = graph.states + static_cast<std::size_t>(current) * b;

    if (BytesEqual(b, packed, found)) return current;

    current = graph.closed[current];
  }

  return -1;
}

__device__
void Close(int node, int *closed, CudaSearchGraph *graph) {
  uint32_t i = graph->hash_values[node] & graph->closed_mask;
  graph->closed[node] = closed[i];
  closed[i] = node;
}

__device__
void Push(int h, int node, CudaSearchGraph *graph, int *next_list,
          int *prev_list) {
  int last = prev_list[h];
  graph->prev[node] = last;
  prev_list[h] = node;
  graph->next[node] = -1;

  if (last == -1)
    next_list[h] = node;
  else
    graph->next[last] = node;
}

__device__
int Pop(int h_max, int *h_min, CudaSearchGraph *graph, int *next_list,
        int *prev_list) {
  for (int i = *h_min; i < h_max; ++i) {
    int node = next_list[i];

    if (node == -1) continue;

    int new_next = graph->next[node];
    next_list[i] = new_next;

    if (new_next == -1) prev_list[i] = -1;

    *h_min = i;

    return node;
  }

  *h_min = h_max;

  return -1;
}

void InitCudaSearchGraph(std::shared_ptr<const SASPlus> problem,
                         std::shared_ptr<const SearchGraph> graph,
                         int closed_exponent, std::size_t gpu_ram,
                         CudaSearchGraph *cuda_graph) {
  cuda_graph->n_variables = problem->n_variables();
  cuda_graph->n_landmarks_bytes = graph->n_landmarks_bytes();
  cuda_graph->closed_mask = (1u << closed_exponent) - 1;
  gpu_ram -= sizeof(int) + 2 * sizeof(uint32_t);

  auto packer = graph->packer();
  cuda_graph->block_size = packer->block_size();
  CudaMallocAndCopy((void**)&cuda_graph->block_index, packer->block_index(),
                    packer->block_index_size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->var_per_block, packer->var_per_block(),
                    packer->var_per_block_size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->shift, packer->shift(),
                    packer->shift_size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->mask, packer->mask(),
                    packer->mask_size() * sizeof(uint32_t));
  gpu_ram -= sizeof(std::size_t);
  gpu_ram -= packer->block_index_size() * sizeof(int);
  gpu_ram -= packer->var_per_block_size() * sizeof(int);
  gpu_ram -= packer->shift_size() * sizeof(int);
  gpu_ram -= packer->mask_size() * sizeof(uint32_t);

  gpu_ram -= sizeof(std::size_t);
  std::size_t node_size = 5 * sizeof(int);
  node_size += (packer->block_size() + 2) * sizeof(uint32_t);
  node_size += graph->n_landmarks_bytes() * sizeof(uint8_t);
  cuda_graph->node_max = gpu_ram / node_size;
  CUDA_CHECK(cudaMalloc((void**)&cuda_graph->actions,
             cuda_graph->node_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&cuda_graph->parents,
             cuda_graph->node_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&cuda_graph->next,
             cuda_graph->node_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&cuda_graph->prev,
             cuda_graph->node_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&cuda_graph->closed,
             cuda_graph->node_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&cuda_graph->states,
             cuda_graph->node_max * graph->state_size()));
  CUDA_CHECK(cudaMalloc((void**)&cuda_graph->hash_values,
             cuda_graph->node_max * sizeof(uint32_t)));
  CUDA_CHECK(cudaMalloc((void**)&cuda_graph->d_hash_values,
             cuda_graph->node_max * sizeof(uint32_t)));

  cuda_graph->landmarks = nullptr;
  std::size_t n_landmarks_bytes = graph->n_landmarks_bytes();

  if (n_landmarks_bytes > 0)
    CUDA_CHECK(cudaMalloc((void**)&cuda_graph->landmarks,
               cuda_graph->node_max * n_landmarks_bytes * sizeof(uint8_t)));
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
