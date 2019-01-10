#include "search/eager_g2bfs.h"

#include <cmath>

#include <vector>

#include "cuda_sas_plus.cuh"
#include "cuda_successor_generator.cuh"
#include "cuda_landmark/cuda_landmark_graph.cuh"

using std::vector;

namespace pplanner {

extern __constant__ CudaSASPlus cuda_problem;
extern __constant__ CudaSuccessorGenerator cuda_generator;
extern __constant__ CudaLandmarkGraph cuda_landmark_graph;

__global__
void Pop(CudaSearchGraph graph, GBFSMessage m, int *next_list, int *prev_list,
         int *closed) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  message.d[id] = 0;
  message.c[id] = 0;

  int *nx = &next_list[id * cuda_open_size];
  int *pv = &prev_list[id * cuda_open_size];
  int *cl = &closed[id * cuda_closed_size];
  int *h_min = &m.h_min[id];

  int node = Pop(cuda_landmark.n_landmarks + 1, h_min, &graph, nx, pv);

  if (node == -1 || GetClosed(graph, closed, ndoe)) return;

  int *s = &m.state[id * cuda_problem.n_variables];
  State(graph, node, s);
  Close(graph, node, closed);

  int c = CountFromTable(cuda_generator, cuda_problem, s);

  m.node[id] = node;
  m.c[id] = c;
}

int PrepareExpansion(int threads, int *h_min, GBFSMessage *m,
                     GBFSMessage *cuda_m) {
  CUDA_CHECK(cudaMemcpy(m->c, cuda_m->c, threads * sizeof(int),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m->h, cuda_m->h_min, threads * sizeof(int),
                        cudaMemcpyDeviceToHost));

  int cumsum = 0;

  for (int i = 0; i < threads; ++i) {
    m->offsets[i] = cumsum;
    int c = m->c[i];
    if (c > 0) ++expanded;
    cumsum += h->c[i];
  }

  if (cumsum > cuda_m->n_successors_max) {
    CudaReallocMessage(cumsum, cuda_m);
    if (cuda_m->h != nullptr) CUDA_CHECK(cudaFree(cuda_m->h));
    CUDA_CHECK(cudaMalloc((void**)&cuda_m->h, cumsum * sizeof(int)));
  }

  CUDA_CHECK(cudaMemcpy(cuda_m->offsets, m->offsets, threads * sizeof(int),
                        cudaMemcpyHostToDevice));

  int local_h_min = INT_MAX;

  for (int i=0; i<threads; ++i) {
    int h = h_message->h[i];
    if (h < local_h_min) local_h_min = h;
  }

  if (local_h_min < *h_min) {
    *h_min = local_h_min;
    std::cout << "new heuristic value for lmcount: " << *h_min << std::endl;
  }

  return cumsum;
}

__global__
void Expand(CudaSearchGraph graph, GBFSMessage m, int *next_list,
            int *prev_list, int *goal) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int c = m.c[id];
  if (c == 0) return;

  int parent = m.node[id];
  const int *s = &message.state[id * cuda_problem.n_variables];
  int offset = message.offsets[id];
  Generate(cuda_generator, cuda_problem, s, &m.actions[offset]);

  int *cs = &message.child[id * cuda_problem.n_variables];
  int *nx = &next_list[id * d_open_size];
  int *pv = &prev_list[id * d_open_size];

  const uint8_t *pac = GetLandmark(graph, parent);
  int8_t *status = &message.status[id * id_max];

  for (int i = offset, n = offset + c; i < n; ++i) {
    int a = m.actions[i];
    memcpy(cs, s, cuda_problem.n_variables * sizeof(int));
    ApplyEffect(cuda_problem, a, child_s);
    uint32_t hash_value = HashByDifference(cuda_closed_hash, cuda_problem, a,
                                           graph.hash_values[parent], s, cs);
    uint32_t d_hash_value = HashByDifference(cuda_distribution_hash,
                                             cuda_problem, a,
                                             graph.d_hash_values[parent], s,
                                             cs);


    int child = d_front + i;
    GenerateNode(child, a, parent, hash_values, d_hash_values, &graph);
    message.child_node[i] = child;
    uint8_t *ac = GetLandmark(graph, child);

    for (int j = 0; j < cuda_landmark_graph.landmark_id_max; ++j)
      ac[j] = 0;

    int h = Evaluate(cuda_landmark_graph, cuda_problem, cs, pc, ac, status);

    if (h == 0) *goal = child;

    int b = d_hash_value % cuda_threads;
    m.num[i] = atomicAdd(&m.d[b], 1);
    m.partition[i] = b;
    m.h[i] = h;
  }
}

void PrepareSort(int threads, GBFSMessage *m GBFSMessage *cuda_m) {
  CUDA_CHECK(cudaMemcpy(m->d, cuda_m->d, threads * sizeof(int),
                        cudaMemcpyDeviceToHost));

  int cumsum = 0;

  for (int i=0; i<threads; ++i) {
    m->pffsets[i] = cumsum;
    int d = m->d[i];
    cumsum += d;
  }

  CUDA_CHECK(cudaMemcpy(cuda_m->pffsets, m->pffsets, threads * sizeof(int),
                        cudaMemcpyHostToDevice));
}

__global__ void SortChildren(const CudaSearchGraph graph, GBFSMessage m) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int offset = m.offsets[id];

  for (int i = offset, n = offset + m.c[id]; i < n; ++i) {
    int b = m.partition[i];
    int idx = m.pffsets[b] + m.num[i];
    m.sorted[idx] = m.child_node[i];
    m.sorted_h[idx] = m.h[i];
  }
}

__global__
void Push(CudaSearchGraph graph, GBFSMessage m, int *next_list,
          int *prev_list, int *closed) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int *nx = &next_list[id * cuda_open_size];
  int *pv = &prev_list[id * cuda_open_size];
  int *cl = &prev_list[id * cuda_closed_size];
  int offset = m.pffsets[id];

  int h_min = m.h_min[id];

  for (int i = offset, n = offset + m.d[id]; i < n; ++i) {
    int h = m.sorted_h[i];
    if (h == -1 || GetClosed(graph, closed, m.sorted[i]) != -1) continue;
    if (h < h_min) h_min = h;
    Push(h, m.sorted[i], &graph, nx, pv);
  }

  m.h_min[id] = h_min;
}

__global__
void PlanStep(const CudaSearchGraph graph, int goal, int *step) {
  *step = 0;
  int current = goal;

  while (graph.actions[current] != -1) {
    ++(*step);
    current = graph.parents[current];
  }
}

__global__ void ExtractPlan(const CudaSearchGraph graph, int step, int goal,
                            int *plan) {
  int n = step;
  int current = goal;

  while (graph.actions[current] != -1) {
    plan[--n] = graph.actions[current];
    current = graph.parents[current];
  }
}

} // namespace rwls

