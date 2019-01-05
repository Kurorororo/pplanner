#ifndef CUDA_LANDMARK_GRAPH_H_
#define CUDA_LANDMARK_GRAPH_H_

#include <memory>

#include "landmark/landmark_graph.h"

namespace pplanner {

struct CudaLandmarkGraph {
  int landmark_id_max;
  int n_bytes;
  int n_landmarks;
  int *vars;
  int *values;
  int *start;
  int *end;
  int *parents;
  int *parent_start;
  int *parent_end;
  int *children;
  int *child_start;
  int *child_end;
  bool *is_goal;
  bool *no_first;
  bool *no_possible;
  bool *is_greedy;
};

extern __constant__ CudaLandmarkGraph cuda_landmark_graph;

__device__
bool IsImplicated(const CudaLandmarkGraph &graph, int i, const int *state);

__device__
inline bool IsGreedy(const CudaLandmarkGraph &graph, int i, int j) {
  return graph.is_greedy[i * graph.landmark_id_max + j];
}

void InitCudaLandmarkGraph(std::shared_ptr<const LandmarkGraph> graph,
                           CudaLandmarkGraph *cuda_graph);

void FreeCudaLandmarkGraph(CudaLandmarkGraph *graph);

} // namespace pplanner

#endif // CUDA_LANDMARK_GRAPH_H_
