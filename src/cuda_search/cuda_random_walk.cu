#include "cuda_search/cuda_random_walk.cuh"

#include "cuda_common/cuda_check.cuh"
#include "cuda_common/cuda_random.cuh"
#include "cuda_landmark/cuda_landmark_count_base.cuh"

namespace pplanner {

__device__
void Fallback(int id, const RandomWalkMessage &m, int *state, uint8_t *accepted,
              int *length) {
  memcpy(state, &m.best_states[id * cuda_problem.n_variables],
         cuda_problem.n_variables * sizeof(int));
  memcpy(accepted, &m.best_accepted[id * cuda_landmark_graph.n_bytes],
         cuda_landmark_graph.n_bytes * sizeof(uint8_t));
  *length = m.best_length[id];
}

__device__
void Swap(int **s, int **c, uint8_t **pc, uint8_t **ac) {
  int *tmp_s = *s;
  *s = *c;
  *c = tmp_s;
  uint8_t *tmp = *pc;
  *pc = *ac;
  *ac = tmp;
}

__global__
void RandomWalk(int walk_length, RandomWalkMessage m) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;

  int n_variables = cuda_problem.n_variables;
  int *s = &m.states[id * 2 * n_variables];
  memcpy(s, &m.best_states[id * n_variables], n_variables * sizeof(int));
  int *c = &m.states[(id * 2 + 1) * n_variables];
  int n_bytes = cuda_landmark_graph.n_bytes;
  uint8_t *pc = &m.accepted[id * 2 * n_bytes];
  memcpy(pc, &m.best_accepted[id * n_bytes], n_bytes * sizeof(uint8_t));
  uint8_t *ac = &m.accepted[(id * 2 + 1) * n_bytes];
  int length = 0;
  m.best_length[id] = 0;
  uint8_t *status = &m.status[id * n_bytes];

  if (m.first_eval[id]) {
    int h = Evaluate(cuda_landmark_graph, cuda_problem, s, pc, ac, &status[id]);
    ++m.evaluated;
    uint8_t *tmp = pc;
    pc = ac;
    ac = tmp;
    m.best_h[id] = h;
  }

  for (int i = 0; i < walk_length; ++i) {
    int a = Sample(cuda_generator, cuda_problem, s, &m.rngs[id]);
    ++m.expanded;

    if (a == -1) {
      Fallback(id, m, s, pc, &length);
      ++m.dead_ends;
      continue;
    }

    ApplyEffect(cuda_problem, a, s, c);

    for (int j = 0; j < n_bytes; ++j)
      ac[j] = 0;

    int h = Evaluate(cuda_landmark_graph, cuda_problem, c, pc, ac, &status[id]);
    ++m.evaluated;

    if (h == -1) {
      Fallback(id, m, s, pc, &length);
      ++m.dead_ends;
      continue;
    }

    m.best_sequences[id * walk_length + length++] = a;

    if (h < m.best_h[id]) {
      m.best_h[id] = h;
      m.best_length[id] = length;
      memcpy(&m.best_states[id * n_variables], c, n_variables * sizeof(int));
      memcpy(&m.best_accepted[id * n_bytes], ac, n_bytes * sizeof(uint8_t));
    }

    Swap(&s, &c, &pc, &ac);
  }
}


void InitRandomWalkMessage(int n_grid, int n_block, int walk_length,
                           int n_variables, int landmark_id_max,
                           RandomWalkMessage *m) {
  int n_threads = n_grid * n_block;
  m->generated = new int[n_threads];
  m->expanded = new int[n_threads];
  m->evaluated = new int[n_threads];
  m->dead_ends = new int[n_threads];
  m->best_h = new int[n_threads];
  m->best_length =  new int[n_threads];
  m->best_states = new int[n_threads * n_variables];
  m->states = new int[n_threads * 2 * n_variables];
  m->best_sequences = new int[n_threads * walk_length];
  int n_bytes = (landmark_id_max + 7) / 8;
  m->best_accepted = new uint8_t[n_threads * n_bytes];
  m->accepted = new uint8_t[n_threads * 2 * n_bytes];
  m->status = new uint8_t[n_threads * landmark_id_max];
  m->first_eval = new bool[n_threads * landmark_id_max];
}

void FreeRandomWalkMessage(RandomWalkMessage *m) {
  delete m->generated;
  delete m->expanded;
  delete m->evaluated;
  delete m->dead_ends;
  delete m->best_h;
  delete m->best_length;
  delete m->best_states;
  delete m->states;
  delete m->best_sequences;
  delete m->best_accepted;
  delete m->accepted;
  delete m->status;
  delete m->first_eval;
}

void CudaInitRandomWalkMessage(int n_grid, int n_block, int walk_length,
                               int n_variables, int landmark_id_max,
                               RandomWalkMessage *m) {
  int n_threads = n_grid * n_block;

  CUDA_CHECK(cudaMalloc((void**)&m->rngs, n_threads * sizeof(curandState_t)));
  SetupStates<<<n_grid, n_block>>>(123, m->rngs);

  CUDA_CHECK(cudaMalloc((void**)&m->generated, n_threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->expanded, n_threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->evaluated, n_threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->dead_ends, n_threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->best_h, n_threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->best_length, n_threads * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->best_states,
                        n_threads * n_variables * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->states,
                        n_threads * 2 * n_variables * sizeof(int)));
  CUDA_CHECK(cudaMalloc((void**)&m->best_sequences,
                        n_threads * walk_length * sizeof(int)));
  int n_bytes = (landmark_id_max + 7) / 8;
  CUDA_CHECK(cudaMalloc((void**)&m->best_accepted,
                        n_threads * n_bytes * sizeof(uint8_t)));
  CUDA_CHECK(cudaMalloc((void**)&m->accepted,
                        n_threads * 2 * n_bytes * sizeof(uint8_t)));
  CUDA_CHECK(cudaMalloc((void**)&m->status,
                        n_threads * landmark_id_max * sizeof(uint8_t)));
  CUDA_CHECK(cudaMalloc((void**)&m->first_eval, n_threads * sizeof(bool)));
}

void CudaFreeRandomWalkMessage(RandomWalkMessage *m) {
  CUDA_CHECK(cudaFree(m->generated));
  CUDA_CHECK(cudaFree(m->expanded));
  CUDA_CHECK(cudaFree(m->evaluated));
  CUDA_CHECK(cudaFree(m->dead_ends));
  CUDA_CHECK(cudaFree(m->best_h));
  CUDA_CHECK(cudaFree(m->best_length));
  CUDA_CHECK(cudaFree(m->best_states));
  CUDA_CHECK(cudaFree(m->states));
  CUDA_CHECK(cudaFree(m->best_sequences));
  CUDA_CHECK(cudaFree(m->best_accepted));
  CUDA_CHECK(cudaFree(m->accepted));
  CUDA_CHECK(cudaFree(m->status));
  CUDA_CHECK(cudaFree(m->first_eval));
}

void Upload(const RandomWalkMessage &m, int n_threads, int n_variables,
            int n_bytes, RandomWalkMessage *cuda_m) {
  CUDA_CHECK(cudaMemcpy(cuda_m->best_h, m.best_h, n_threads * sizeof(int),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(cuda_m->best_states, m.best_states,
                        n_threads * n_variables * sizeof(int),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(cuda_m->best_accepted, m.best_accepted,
                        n_threads * n_bytes * sizeof(int),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(cuda_m->first_eval, m.first_eval,
                        n_threads * sizeof(bool), cudaMemcpyHostToDevice));
}

void Download(const RandomWalkMessage &cuda_m, int n_threads, int n_variables,
              int n_bytes, int walk_length, RandomWalkMessage *m) {
  CUDA_CHECK(cudaMemcpy(m->best_h, cuda_m.best_h, n_threads * sizeof(int),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m->best_length, cuda_m.best_length,
                        n_threads * sizeof(int), cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m->best_states, cuda_m.best_states,
                        n_threads * n_variables * sizeof(int),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m->best_accepted, cuda_m.best_accepted,
                        n_threads * n_bytes * sizeof(int),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m->best_sequences, cuda_m.best_sequences,
                        n_threads * walk_length * sizeof(int),
                        cudaMemcpyDeviceToHost));
}

void UploadStatistics(RandomWalkMessage &m, int n_threads,
                      RandomWalkMessage *cuda_m) {
  for (int i = 0; i < n_threads; ++i) {
    m.generated[i] = 0;
    m.expanded[i] = 0;
    m.evaluated[i] = 0;
    m.dead_ends[i] = 0;
  }

  CUDA_CHECK(cudaMemcpy(cuda_m->generated, m.generated,
                        n_threads * sizeof(int), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(cuda_m->expanded, m.expanded,
                        n_threads * sizeof(int), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(cuda_m->evaluated, m.evaluated,
                        n_threads * sizeof(int), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(cuda_m->dead_ends, m.dead_ends,
                        n_threads * sizeof(int), cudaMemcpyHostToDevice));
}

void DownloadStatistics(const RandomWalkMessage &cuda_m, int n_threads,
                        RandomWalkMessage *m) {
  CUDA_CHECK(cudaMemcpy(m->generated, cuda_m.generated,
                        n_threads * sizeof(int), cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m->expanded, cuda_m.expanded,
                        n_threads * sizeof(int), cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m->evaluated, cuda_m.evaluated,
                        n_threads * sizeof(int), cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(m->dead_ends, cuda_m.dead_ends,
                        n_threads * sizeof(int), cudaMemcpyDeviceToHost));
}

} // namespace pplanner
