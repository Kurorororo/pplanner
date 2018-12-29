#ifndef CUDA_RANDOM_WALK_H_
#define CUDA_RANDOM_WALK_H_

#include "cuda_sas_plus.cuh"
#include "cuda_successor_generator.cuh"

namespace pplanner {

struct RandomWalkMessage {
  int *generated;
  int *expanded;
  int *evaluated;
  int *dead_ends;
  int *best_h;
  int *best_length;
  int *best_states;
  int *states;
  int *best_sequences;
  uint8_t *best_accepted;
  uint8_t *accepted;
  uint8_t *status;
  curandState *rngs;
};

__global__
void RandomWalk(int walk_length, RandomWalkMessage m);

void InitRandomWalkMessage(int n_grid, int n_block, int walk_length,
                           int n_variables, int landmark_id_max,
                           RandomWalkMessage *m);

void FreeRandomWalkMessage(RandomWalkMessage *m);

void CudaInitRandomWalkMessage(int n_grid, int n_block, int walk_length,
                               int n_variables, int landmark_id_max,
                               RandomWalkMessage *m);

void CudaFreeRandomWalkMessage(RandomWalkMessage *m);

void Uplaod(const RandomWalkMessage &m, int n_threads, int n_variables,
            int n_bytes, RandomWalkMessage *cuda_m);

void Download(const RandomWalkMessage &cuda_m, RandomWalkMessage *m);

void UploadStatistics(RandomWalkMessage &m, int n_threads,
                      RandomWalkMessage *cuda_m);

void DownloadStatistics(const RandomWalkMessage &cuda_m, int n_threds,
                        RandomWalkMessage *m);

} // namespace pplanner

#endif // CUDA_RANDOM_WALK_H_
