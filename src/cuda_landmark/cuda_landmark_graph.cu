#include "cuda_landmark/cuda_landmark_graph.cuh"

#include "cuda_common/cuda_check.cuh"

namespace pplanner {

__constant__ CudaLandmarkGraph cuda_landmark_graph;

__device__
bool IsImplicated(const CudaLandmarkGraph &graph, int i, const int *state) {
  for (int j = graph.start[i], n = graph.end[i]; j < n; ++j)
    if (state[graph.vars[j]] == graph.values[j]) return true;

  return false;
}

void InitCudaLandmarkGraph(std::shared_ptr<const LandmarkGraph> graph,
                           CudaLandmarkGraph *cuda_graph) {
  int landmark_id_max = graph->landmark_id_max();
  cuda_graph->landmark_id_max = landmark_id_max;
  cuda_graph->n_bytes = (landmark_id_max + 7) / 8;
  cuda_graph->n_landmarks = graph->n_landmarks();

  std::vector<int> vars;
  std::vector<int> values;
  std::vector<int> start;
  std::vector<int> end;

  for (auto &lm : graph->GetLandmarks()) {
    start.push_back(vars.size());

    for (int i = 0, n = lm.size(); i < n; ++i) {
      vars.push_back(lm.Var(i));
      values.push_back(lm.Value(i));
    }

    end.push_back(vars.size());
  }

  CudaMallocAndCopy((void**)&cuda_graph->vars, vars.data(),
                    vars.size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->values, values.data(),
                    values.size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->start, start.data(),
                    start.size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->end, end.data(),
                    end.size() * sizeof(int));

  std::vector<int> children;
  std::vector<int> child_start;
  std::vector<int> child_end;

  for (int i = 0; i < landmark_id_max; ++i) {
    auto lms = graph->GetTermIdsByInitId(i);
    child_start.push_back(children.size());
    children.insert(children.end(), lms.begin(), lms.end());
    child_end.push_back(children.size());
  }

  CudaMallocAndCopy((void**)&cuda_graph->children, children.data(),
                    children.size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->child_start, child_start.data(),
                    child_start.size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->child_end, child_end.data(),
                    child_end.size() * sizeof(int));

  std::vector<int> parents;
  std::vector<int> parent_start;
  std::vector<int> parent_end;

  for (int i = 0; i < landmark_id_max; ++i) {
    auto lms = graph->GetInitIdsByTermId(i);
    parent_start.push_back(parents.size());
    parents.insert(parents.end(), lms.begin(), lms.end());
    parent_end.push_back(parents.size());
  }

  CudaMallocAndCopy((void**)&cuda_graph->parents, parents.data(),
                    parents.size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->parent_start, parent_start.data(),
                    parent_start.size() * sizeof(int));
  CudaMallocAndCopy((void**)&cuda_graph->parent_end, parent_end.data(),
                    parent_end.size() * sizeof(int));

  bool is_goal[landmark_id_max];
  bool no_possible[landmark_id_max];
  bool no_first[landmark_id_max];

  for (int i = 0; i < landmark_id_max; ++i) {
    is_goal[i] = graph->IsGoal(i);
    no_possible[i] = graph->GetPossibleAchieversSize(i) == 0;
    no_first[i] = graph->GetFirstAchieversSize(i) == 0;
  }

  CudaMallocAndCopy((void**)&cuda_graph->is_goal, is_goal,
                    landmark_id_max * sizeof(bool));
  CudaMallocAndCopy((void**)&cuda_graph->no_possible, no_possible,
                    landmark_id_max * sizeof(bool));
  CudaMallocAndCopy((void**)&cuda_graph->no_first, no_first,
                    landmark_id_max * sizeof(bool));

  bool is_greedy[landmark_id_max * landmark_id_max];

  for (int i = 0; i < landmark_id_max; ++i) {
    for (int j = 0; j < landmark_id_max; ++j) {
      bool is = graph->IsAdjacent(i, j)
        && graph->GetOrderingType(i, j) == LandmarkGraph::GREEDY;
      is_greedy[i * landmark_id_max + j] = is;
    }
  }

  CudaMallocAndCopy((void**)&cuda_graph->is_greedy, is_greedy,
                    landmark_id_max * landmark_id_max * sizeof(bool));
}

void FreeCudaLandmarkGraph(CudaLandmarkGraph *graph) {
  CUDA_CHECK(cudaFree(graph->vars));
  CUDA_CHECK(cudaFree(graph->values));
  CUDA_CHECK(cudaFree(graph->start));
  CUDA_CHECK(cudaFree(graph->end));
  CUDA_CHECK(cudaFree(graph->parents));
  CUDA_CHECK(cudaFree(graph->parent_start));
  CUDA_CHECK(cudaFree(graph->parent_end));
  CUDA_CHECK(cudaFree(graph->children));
  CUDA_CHECK(cudaFree(graph->child_start));
  CUDA_CHECK(cudaFree(graph->child_end));
  CUDA_CHECK(cudaFree(graph->is_goal));
  CUDA_CHECK(cudaFree(graph->no_first));
  CUDA_CHECK(cudaFree(graph->no_possible));
  CUDA_CHECK(cudaFree(graph->is_greedy));
}

} // namespace pplanner
