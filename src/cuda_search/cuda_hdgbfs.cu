#include "cuda_search/cuda_hdgbfs.cuh"

#include "evaluator.h"
#include "open_list_factory.h"
#include "cuda_common/cuda_check.cuh"
#include "cuda_search/cuda_random_walk.cuh"

namespace pplanner {

using std::vector;

extern __constant__ CudaSASPlus cuda_problem;
extern __constant__ CudaSuccessorGenerator cuda_generator;
extern __constant__ CudaLandmarkGraph cuda_landmark_graph;
extern __constant__ CudaZobristHash cuda_c_hash;
extern __constant__ CudaZobristHash cuda_d_hash;

void CudaHDGBFS::Init(const boost::property_tree::ptree &pt) {
  if (auto opt = pt.get_optional<int>("n_grid"))
    n_grid_ = opt.get();

  if (auto opt = pt.get_optional<int>("n_block"))
    n_block_ = opt.get();

  n_threads_ = n_grid_ * n_block_;

  graph_->InitLandmarks(lmcount_->landmark_graph());

  std::size_t gpu_ram = 8000000000;

  if (auto opt = pt.get_optional<size_t>("gpu_ram"))
    gpu_ram = opt.get();

  gpu_ram -= InitCudaSASPlus(problem_, cuda_problem_);
  gpu_ram -= InitCudaSuccessorGenerator(generator_, cuda_generator_);
  gpu_ram -= InitCudaLandmarkGraph(lmcount_->landmark_graph(),
                                   cuda_landmark_graph_);
  gpu_ram -= InitCudaZobristHash(c_hash_, cuda_c_hash_);
  gpu_ram -= InitCudaZobristHash(d_hash_, cuda_d_hash_);

  CUDA_CHECK(cudaMemcpyToSymbol(cuda_problem, cuda_problem_,
                                sizeof(CudaSASPlus)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_generator, cuda_generator_,
                                sizeof(CudaSuccessorGenerator)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_landmark_graph, cuda_landmark_graph_,
                                sizeof(CudaLandmarkGraph)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_c_hash, cuda_c_hash_,
                                sizeof(CudaZobristHash)));
  CUDA_CHECK(cudaMemcpyToSymbol(cuda_d_hash, cuda_d_hash_,
                                sizeof(CudaZobristHash)));

  gpu_ram -= CudaInitializeOpenList(n_threads_,
                                    lmcount_->landmark_graph()->n_landmarks(),
                                    &open_);

  int n_successors_max = 10 * n_threads_;
  gpu_ram -= 10 * n_threads_ * sizeof(int);
  InitializeGBFSMessage(n_threads_, &m_);
  gpu_ram -= CudaInitializeGBFSMessage(problem_, graph_, n_threads_,
                                       n_successors_max, &cuda_m_);
  int closed_exponent = 13;
  std::size_t closed_size = 1u << closed_exponent;
  gpu_ram -= CudaInitializeClosedList(n_threads_, closed_size, &closed_);
  gpu_ram -= 1000000000;
  InitCudaSearchGraph(problem_, graph_, closed_exponent, gpu_ram, &cuda_graph_);
}

CudaHDGBFS::~CudaHDGBFS() {
  FreeCudaSASPlus(cuda_problem_);
  FreeCudaSuccessorGenerator(cuda_generator_);
  FreeCudaLandmarkGraph(cuda_landmark_graph_);
  FreeCudaZobristHash(cuda_c_hash_);
  FreeCudaZobristHash(cuda_d_hash_);
  FreeCudaSearchGraph(&cuda_graph_);
  FreeGBFSMessage(&m_);
  CudaFreeGBFSMessage(&cuda_m_);
  delete cuda_problem_;
  delete cuda_generator_;
  delete cuda_landmark_graph_;
  delete cuda_c_hash_;
  delete cuda_d_hash_;
}

int CudaHDGBFS::Search() {
  CUDA_CHECK(cudaMemcpy(cuda_m_.states, problem_->initial().data(),
                        problem_->n_variables() * sizeof(int),
                        cudaMemcpyHostToDevice));
  int *cuda_h = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_h, sizeof(int)));
  InitialEvaluate<<<1, 1>>>(cuda_graph_, cuda_m_, open_, n_threads_, cuda_h);
  m_.n_nodes += 1;
  cuda_m_.n_nodes += 1;
  int min_h = -1;
  CUDA_CHECK(cudaMemcpy(&min_h, cuda_h, sizeof(int),
                        cudaMemcpyDeviceToHost));
  std::cout << "Initial h value: " << min_h << std::endl;
  int goal = -1;
  int *cuda_goal = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_goal, sizeof(int)));
  CUDA_CHECK(cudaMemcpy(cuda_goal, &goal, sizeof(int), cudaMemcpyHostToDevice));

  while (goal == -1) {
    Pop<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_, open_, closed_);
    expanded_ += PrepareExpansion(n_threads_, &m_, &cuda_m_);
    Expand<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_, n_threads_, cuda_goal);
    CUDA_CHECK(cudaMemcpy(&goal, cuda_goal, sizeof(int),
                          cudaMemcpyDeviceToHost));

    if (goal != -1) break;

    generated_ += PrepareSort(n_threads_, &m_, &cuda_m_);

    if (generated_ >= static_cast<int>(cuda_graph_.node_max - 1)) {
      std::cerr << "exceeded GPU RAM limit" << std::endl;
      CUDA_CHECK(cudaFree(cuda_goal));
      return -1;
    }

    SortChildren<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_);
    CUDA_CHECK(cudaDeviceSynchronize());
    Push<<<n_grid_, n_block_>>>(cuda_graph_, cuda_m_, closed_, open_);
    int h = -1;
    CUDA_CHECK(cudaMemcpy(&h, cuda_m_.h, sizeof(int), cudaMemcpyDeviceToHost));

    if (h != -1 && h < min_h) {
      min_h = h;
      std::cout << "New best heuristic value: " << min_h << std::endl;
      std::cout << "[" << expanded_ << " expanded]" << std::endl;
    }
  }

  CUDA_CHECK(cudaFree(cuda_goal));

  return goal;
}

std::vector<int> CudaHDGBFS::ExtractPlan(int node) {
  if (node == -1) return std::vector<int>{-1};

  int *goals = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&goals, sizeof(int)));
  CUDA_CHECK(cudaMemcpy(goals, &node, sizeof(int), cudaMemcpyHostToDevice));
  int *cuda_steps = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_steps, sizeof(int)));
  NPlanStep<<<1, 1>>>(cuda_graph_, goals, cuda_steps);
  int step;
  CUDA_CHECK(cudaMemcpy(&step, cuda_steps, sizeof(int),
                        cudaMemcpyDeviceToHost));
  int *cuda_offsets = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_offsets, sizeof(int)));
  int offsets = step;
  CUDA_CHECK(cudaMemcpy(cuda_offsets, &offsets,  sizeof(int),
                        cudaMemcpyHostToDevice));
  int *cuda_plan = nullptr;
  CUDA_CHECK(cudaMalloc((void**)&cuda_plan, step * sizeof(int)));
  CudaExtractPlan<<<1, 1>>>(cuda_graph_, cuda_offsets, goals, cuda_plan);
  std::vector<int> plan(step);
  CUDA_CHECK(cudaMemcpy(plan.data(), cuda_plan, step * sizeof(int),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaFree(goals));
  CUDA_CHECK(cudaFree(cuda_steps));
  CUDA_CHECK(cudaFree(cuda_offsets));
  CUDA_CHECK(cudaFree(cuda_plan));

  return plan;
}

void CudaHDGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
}

} // namespace pplanner
