#include "cuda_landmark/cuda_landmark_count_base.cuh"

namespace pplanner {

__device__
int ReachedSize(const CudaLandmarkGraph &graph const uint8_t *parent_accepted,
                uint8_t *accepted) {
  int size = 0;

  for (int psi_id = 0, n = graph.landmark_id_max; psi_id < n; ++psi_id) {
    if (CudaGet(parent_accepted, psi_id)) {
      Up(accepted, psi_id);
      ++size;
      continue;
    }

    if (IsImplicated(graph, psi_id, state)) {
      Up(accepted, psi_id);
      ++size;
    }
  }

  return size;
}

__device__
int NeededSize(const CudaLandmarkGraph &graph, const int *state,
               const uint8_t *accepted, uint8_t *status) {
  int size = 0;

  for (int phi_id = 0, n = graph.landmark_id_max; phi_id < n; ++phi_id) {
    if (!Get(accepted, phi_id)) continue;
    status[phi_id] = 1;

    if (IsImplicated(graph, phi_id, state)) continue;

    if (graph.is_goal[phi_id]) {
      status[phi_id] = 2;
      ++size;
      continue;
    }

    int start = graph.child_start[phi_id];
    int end = graph.children_end[phi_id];

    for (int psi_id = start; psi_id < end; ++psi_id) {
      if (IsGreedy(graph, phi_id, psi_id) && !CudaGet(accepted, psi_id)) {
        status[phi_id] = 2;
        ++size;
        break;
      }
    }
  }

  return size;
}

__device__
int Evaluate(const CudaLandmarkGraph &graph, const CudaSASPlus &problem,
             const int *state, const uint8_t *parent_accepted,
             uint8_t *accepted, uint8_t *status) {
  if (IsGoal(problem, state)) return 0;

  for (int i = 0, n = graph.landmark_id_max; i < n; ++i)
    status[i] = 0;

  int reached_size = 0;

  if (parent_accepted == nullptr) {
    for (int i = 0, n = graph.landmark_id_max; i < n; ++i) {
      if (IsImplicated(graph, i, state)
          && graph.parent_start[i] == graph.parent_end[i]) {
        status[i] = 1;
        CudaUp(accepted, i);
        ++reached_size;
      }
    }
  } else {
    reached_size = ReachedSize(graph, state, parent_accepted, accepted);
  }

  int needed_size = NeededSize(graph, state, accepted, status);

  for (int i = 0, n = graph.landmark_id_max; i < n; ++i) {
    if ((status[i] == 0 && graph.no_first[i])
        || (status[i] == 2 && graph.no_possible[i])) return -1;
  }

  return graph.n_landmarks - reached_size + needed_size;
}

} // namespace pplanner
