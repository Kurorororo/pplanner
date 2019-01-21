#include "cuda_search/cuda_gbfs_kernel.cuh"

#include <vector>

#include "cuda_sas_plus.cuh"
#include "cuda_successor_generator.cuh"
#include "cuda_common/cuda_check.cuh"
#include "cuda_landmark/cuda_landmark_count_base.cuh"
#include "cuda_landmark/cuda_landmark_graph.cuh"

using std::vector;

namespace pplanner {

extern __constant__ CudaSASPlus cuda_problem;
extern __constant__ CudaSuccessorGenerator cuda_generator;
extern __constant__ CudaLandmarkGraph cuda_landmark_graph;
__constant__ CudaZobristHash cuda_c_hash;
__constant__ CudaZobristHash cuda_d_hash;

__global__
void CudaInitialEvaluate(CudaSearchGraph graph, GBFSMessage m,
                         CudaOpenList open, int n_thread, int *h) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int *s = m.states;
  uint32_t d_hash = Hash(cuda_d_hash, cuda_problem, s);
  int proc = d_hash % n_thread;

  if (id == proc) {
    uint8_t *ac = m.accepted;
    uint8_t *status = m.status;
    uint32_t *packed = m.packed;

    for (int j = 0, n = cuda_landmark_graph.landmark_id_max; j < n; ++j)
      ac[j] = 0;

    *h = Evaluate(cuda_landmark_graph, cuda_problem, s, nullptr, ac, status);

    if (*h == -1 || *h == 0) return;

    Pack(graph, s, packed);

    uint32_t c_hash = Hash(cuda_c_hash, cuda_problem, s);

    GenerateNode(0, -1, -1, c_hash, d_hash, packed, &graph);
    SetLandmark(0, cuda_landmark_graph.n_bytes, ac, &graph);

    int *nx = &open.next[proc * open.n_unit];
    int *pv = &open.prev[proc * open.n_unit];

    Push(*h, 0, &graph, nx, pv);
    m.h_min[id] = *h;
  } else {
    m.h_min[id] = -1;
  }
}

__global__
void CudaInitialPush(CudaSearchGraph graph, GBFSMessage m, CudaOpenList open,
                     int n_thread) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int *s = m.states;
  uint32_t d_hash = Hash(cuda_d_hash, cuda_problem, s);
  int proc = d_hash % n_thread;

  if (id == proc) {
    uint32_t c_hash = Hash(cuda_c_hash, cuda_problem, s);
    Pack(graph, s, m.packed);
    GenerateNode(0, -1, -1, c_hash, d_hash, m.packed, &graph);
    SetLandmark(0, cuda_landmark_graph.n_bytes, m.accepted, &graph);

    int *nx = &open.next[proc * open.n_unit];
    int *pv = &open.prev[proc * open.n_unit];

    Push(m.h_min[0], 0, &graph, nx, pv);
    m.h_min[id] = m.h_min[0];
  } else {
    m.h_min[id] = -1;
  }
}

__global__
void CudaPop(CudaSearchGraph graph, GBFSMessage m, CudaOpenList open,
             CudaClosedList closed) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  m.received_counts[id] = 0;
  m.successor_counts[id] = 0;

  int *nx = &open.next[id * open.n_unit];
  int *pv = &open.prev[id * open.n_unit];
  int *cl = &closed.closed[id * closed.n_unit];
  int *h_min = &m.h_min[id];

  int node = Pop(cuda_landmark_graph.n_landmarks, h_min, &graph, nx, pv);

  if (node == -1 || GetClosed(graph, cl, node) != -1) return;

  int *s = &m.states[id * cuda_problem.n_variables];
  State(graph, node, s);
  Close(node, cl, &graph);
  m.successor_counts[id] = Count(cuda_generator, cuda_problem, s);
  m.nodes[id] = node;
}

int PrepareExpansion(int threads, GBFSMessage *m, GBFSMessage *cuda_m) {
  CUDA_CHECK(cudaMemcpy(m->successor_counts, cuda_m->successor_counts,
                        threads * sizeof(int), cudaMemcpyDeviceToHost));

  int prefix_sum = 0;
  int expanded = 0;

  for (int i = 0; i < threads; ++i) {
    m->successor_offsets[i] = prefix_sum;
    int c = m->successor_counts[i];
    if (c > 0) ++expanded;
    prefix_sum += m->successor_counts[i];
  }

  if (prefix_sum > cuda_m->n_successors_max) {
    CudaReallocMessage(prefix_sum, cuda_m);
    m->n_successors_max = prefix_sum;
    cuda_m->n_successors_max = prefix_sum;
  }

  CUDA_CHECK(cudaMemcpy(cuda_m->successor_offsets, m->successor_offsets,
                        threads * sizeof(int), cudaMemcpyHostToDevice));

  return expanded;
}

__global__
void CudaExpand(CudaSearchGraph graph, GBFSMessage m, int n_threads,
                int *goal) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int c = m.successor_counts[id];
  if (c == 0) return;

  int parent = m.nodes[id];
  const int *s = &m.states[id * cuda_problem.n_variables];
  int offset = m.successor_offsets[id];
  Generate(cuda_generator, cuda_problem, s, &m.actions[offset]);

  int *cs = &m.child_states[id * cuda_problem.n_variables];
  uint32_t *packed = &m.packed[id * graph.block_size];

  const uint8_t *pc = GetLandmark(graph, parent);
  uint8_t *ac = &m.accepted[id * cuda_landmark_graph.n_bytes];
  uint8_t *status = &m.status[id * cuda_landmark_graph.landmark_id_max];

  for (int i = offset, n = offset + c; i < n; ++i) {
    int a = m.actions[i];
    memcpy(cs, s, cuda_problem.n_variables * sizeof(int));
    ApplyEffect(cuda_problem, a, s, cs);
    uint32_t c_hash = HashByDifference(cuda_c_hash, cuda_problem, a,
                                       graph.hash_values[parent], s, cs);
    Pack(graph, cs, packed);

    for (int j = 0, n = cuda_landmark_graph.n_bytes; j < n; ++j)
      ac[j] = 0;

    int h = Evaluate(cuda_landmark_graph, cuda_problem, cs, pc, ac, status);
    uint32_t d_hash = HashByDifference(cuda_d_hash, cuda_problem, a,
                                       graph.d_hash_values[parent], s, cs);
    int child = m.n_nodes + i;
    m.successors[i] = child;

    if (child >= graph.node_max)
      continue;

    GenerateNode(child, a, parent, c_hash, d_hash, packed, &graph);
    SetLandmark(child, cuda_landmark_graph.n_bytes, ac, &graph);

    if (h == 0) *goal = child;

    int proc = d_hash % n_threads;
    m.procs[i] = proc;
    m.received_indices[i] = atomicAdd(&m.received_counts[proc], 1);
    m.h[i] = h;
  }
}

int PrepareSort(int threads, GBFSMessage *m, GBFSMessage *cuda_m) {
  CUDA_CHECK(cudaMemcpy(m->received_counts, cuda_m->received_counts,
                        threads * sizeof(int), cudaMemcpyDeviceToHost));

  int prefix_sum = 0;

  for (int i = 0; i < threads; ++i) {
    m->received_offsets[i] = prefix_sum;
    prefix_sum += m->received_counts[i];
  }

  m->n_nodes += prefix_sum;
  cuda_m->n_nodes += prefix_sum;

  CUDA_CHECK(cudaMemcpy(cuda_m->received_offsets, m->received_offsets,
                        threads * sizeof(int), cudaMemcpyHostToDevice));

  return prefix_sum;
}

__global__
void CudaSortChildren(const CudaSearchGraph graph, GBFSMessage m) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;
  int offset = m.successor_offsets[id];

  for (int i = offset, n = offset + m.successor_counts[id]; i < n; ++i) {
    int idx = m.received_offsets[m.procs[i]] + m.received_indices[i];
    m.sorted[idx] = m.successors[i];
    m.sorted_h[idx] = m.h[i];
  }
}

__global__
void CudaPush(CudaSearchGraph graph, GBFSMessage m, CudaClosedList closed,
              CudaOpenList open) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int *nx = &open.next[id * open.n_unit];
  int *pv = &open.prev[id * open.n_unit];
  int *cl = &closed.closed[id * closed.n_unit];
  int offset = m.received_offsets[id];
  int h_min = m.h_min[id];

  for (int i = offset, n = offset + m.received_counts[id]; i < n; ++i) {
    int h = m.sorted_h[i];
    int node = m.sorted[i];
    if (h == -1 || GetClosed(graph, cl, node) != -1) continue;
    if (h < h_min || h_min == -1) h_min = h;
    Push(h, node, &graph, nx, pv);
  }

  m.h_min[id] = h_min;
}

__global__
void CudaNPlanStep(const CudaSearchGraph graph, int *goals, int *steps) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int step = 0;
  int current = goals[id];

  if (current == -1) return;

  while (graph.actions[current] != -1) {
    ++step;
    current = graph.parents[current];
  }

  steps[id] = step;
}

__global__
void CudaExtractPlan(const CudaSearchGraph graph, int *offsets, int *goals,
                     int *plans) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int n = offsets[id];
  int current = goals[id];

  if (current == -1) return;

  while (graph.actions[current] != -1) {
    plans[--n] = graph.actions[current];
    current = graph.parents[current];
  }
}

void InitializeGBFSMessage(int threads, GBFSMessage *m) {
  m->n_nodes = 0;
  m->successor_counts = new int[threads];
  m->received_counts = new int[threads];
  m->successor_offsets = new int[threads];
  m->received_offsets = new int[threads];
}

void FreeGBFSMessage(GBFSMessage *m) {
  delete[] m->successor_counts;
  delete[] m->successor_offsets;
  delete[] m->received_counts;
  delete[] m->received_offsets;
}

std::size_t CudaInitializeGBFSMessage(
    std::shared_ptr<const SASPlus> problem,
    std::shared_ptr<const SearchGraphWithLandmarks> graph,
    int threads,
    int n_successors_max,
    GBFSMessage *m) {
  m->n_successors_max = n_successors_max;
  m->n_nodes = 0;
  std::size_t size = 2 * sizeof(int);

  CUDA_CHECK(cudaMalloc((void**)&m->nodes, threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->successor_counts, threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->successor_offsets, threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->received_counts, threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->received_offsets, threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->h_min, threads * sizeof(int)));
  size += 6 * threads * sizeof(int);

  CUDA_CHECK(cudaMalloc((void**)&m->states,
                        threads * problem->n_variables() * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->child_states,
                        threads * problem->n_variables() * sizeof(int)));
  size += 2 * threads * problem->n_variables() * sizeof(int);

  int n_bytes = graph->n_landmarks_bytes();
  int landmark_id_max = graph->landmark_id_max();
  CUDA_CHECK(cudaMalloc((void**)&m->accepted,
                        threads * n_bytes * sizeof(uint8_t)));
  CUDA_CHECK(cudaMalloc((void**)&m->status,
                        threads * landmark_id_max * sizeof(uint8_t)));
  size += threads * (n_bytes + landmark_id_max) * sizeof(uint8_t);
  CUDA_CHECK(cudaMalloc((void**)&m->packed, threads * graph->state_size()));
  size += threads * graph->state_size();

  CUDA_CHECK(cudaMalloc((void**)&m->actions, n_successors_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->successors,
                        n_successors_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->h, n_successors_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->procs, n_successors_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->received_indices,
                        n_successors_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->sorted, n_successors_max * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->sorted_h, n_successors_max * sizeof(int)));
  size += 7 * n_successors_max * sizeof(int);

  return size;
}

void CudaFreeGBFSMessage(GBFSMessage *m) {
  CUDA_CHECK(cudaFree(m->nodes));
  CUDA_CHECK(cudaFree(m->successor_counts));
  CUDA_CHECK(cudaFree(m->successor_offsets));
  CUDA_CHECK(cudaFree(m->received_counts));
  CUDA_CHECK(cudaFree(m->received_offsets));
  CUDA_CHECK(cudaFree(m->states));
  CUDA_CHECK(cudaFree(m->child_states));
  CUDA_CHECK(cudaFree(m->accepted));
  CUDA_CHECK(cudaFree(m->status));
  CUDA_CHECK(cudaFree(m->packed));
  CUDA_CHECK(cudaFree(m->actions));
  CUDA_CHECK(cudaFree(m->successors));
  CUDA_CHECK(cudaFree(m->h));
  CUDA_CHECK(cudaFree(m->procs));
  CUDA_CHECK(cudaFree(m->sorted));
  CUDA_CHECK(cudaFree(m->sorted_h));
}

void CudaReallocMessage(int size, GBFSMessage *m) {
  CUDA_CHECK(cudaFree(m->actions));
  CUDA_CHECK(cudaFree(m->successors));
  CUDA_CHECK(cudaFree(m->h));
  CUDA_CHECK(cudaFree(m->procs));
  CUDA_CHECK(cudaFree(m->received_indices));
  CUDA_CHECK(cudaFree(m->sorted));
  CUDA_CHECK(cudaFree(m->sorted_h));

  CUDA_CHECK(cudaMalloc((void**)&m->actions, size * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->successors, size * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->h, size * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->procs, size * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->received_indices, size * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->sorted, size * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->sorted_h, size * sizeof(int)));
}


std::size_t CudaInitializeOpenList(int threads, std::size_t size,
                                   CudaOpenList *open) {
  open->n_unit = size;
  std::vector<int> cpu_list(threads * size, -1);
  CudaMallocAndCopy((void**)&open->next, cpu_list.data(),
                    threads * size * sizeof(int));
  CudaMallocAndCopy((void**)&open->prev, cpu_list.data(),
                    threads * size * sizeof(int));

  return 2 * threads * size * sizeof(int);
}

void CudaClearOpenList(int threads, CudaOpenList *open) {
  std::size_t size = threads * open->n_unit;
  std::vector<int> cpu_list(size, -1);
  CUDA_CHECK(cudaMemcpy(open->next, cpu_list.data(), size * sizeof(int),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(open->prev, cpu_list.data(), size * sizeof(int),
                        cudaMemcpyHostToDevice));
}

void CudaFreeOpenList(CudaOpenList *open) {
  CUDA_CHECK(cudaFree(open->next));
  CUDA_CHECK(cudaFree(open->prev));
}

std::size_t CudaInitializeClosedList(int threads, std::size_t size,
                                     CudaClosedList *closed) {
  closed->n_unit = size;
  std::vector<int> cpu_list(threads * size, -1);
  CudaMallocAndCopy((void**)&closed->closed, cpu_list.data(),
                    threads * size * sizeof(int));

  return threads * size * sizeof(int);
}

void CudaClearClosedList(int threads, CudaClosedList *closed) {
  std::size_t size = threads * closed->n_unit;
  std::vector<int> cpu_list(size, -1);
  CUDA_CHECK(cudaMemcpy(closed->closed, cpu_list.data(), size * sizeof(int),
                        cudaMemcpyHostToDevice));
}

void CudaFreeClosedList(CudaClosedList *closed) {
  CUDA_CHECK(cudaFree(closed->closed));
}

} // namespace rwls

