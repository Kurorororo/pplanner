#ifndef CUDA_HETRO_GBFS_H_
#define CUDA_HETRO_GBFS_H_

#include <memory>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "cuda_sas_plus.cuh"
#include "cuda_search_graph.cuh"
#include "cuda_successor_generator.cuh"
#include "open_list.h"
#include "sas_plus.h"
#include "search.h"
#include "successor_generator.h"
#include "cuda_landmark/cuda_landmark_graph.cuh"
#include "cuda_search/cuda_gbfs_kernel.cuh"
#include "cuda_hash/cuda_zobrist_hash.cuh"
#include "landmark/landmark_count_base.h"

namespace pplanner {

class CudaHetroGBFS : public Search {
 public:
  CudaHetroGBFS(std::shared_ptr<const SASPlus> problem,
           const boost::property_tree::ptree &pt)
    : n_grid_(20),
      n_block_(128),
      generated_(0),
      expanded_(0),
      goal_on_gpu_(false),
      cuda_steps_(nullptr),
      cuda_offsets_(nullptr),
      problem_(problem),
      generator_(std::make_shared<SuccessorGenerator>(problem)),
      lmcount_(std::make_shared<LandmarkCountBase>(
            problem, false, false, false, false)),
      open_(nullptr),
      d_hash_(std::make_shared<ZobristHash>(problem, 2886379259)),
      graph_(std::make_shared<SearchGraphWithLandmarks>(problem, 22)),
      cuda_problem_(new CudaSASPlus),
      cuda_generator_(new CudaSuccessorGenerator),
      cuda_landmark_graph_(new CudaLandmarkGraph),
      cuda_c_hash_(new CudaZobristHash),
      cuda_d_hash_(new CudaZobristHash) { Init(pt); }

  ~CudaHetroGBFS();

  std::vector<int> Plan() override {
    int goal = Search();

    return ExtractPlan(goal);
  }

  void DumpStatistics() const override;

 private:
  void Init(const boost::property_tree::ptree &pt);

  std::size_t AllocateMessage();

  int Search();

  void InitialEvaluateAndPush();

  void InitialPushGPU(int node, int h);

  int CpuExpand();

  void ClearGPU();

  std::vector<int> ExtractPlan(int node);

  int n_grid_;
  int n_block_;
  int n_threads_;
  int generated_;
  int expanded_;
  int best_h_;
  int best_node_;
  int gpu_start_;
  bool goal_on_gpu_;
  int *cuda_steps_;
  int *cuda_offsets_;
  std::unordered_map<int, std::vector<int> > sequences_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<LandmarkCountBase> lmcount_;
  std::shared_ptr<ZobristHash> d_hash_;
  std::shared_ptr<SearchGraphWithLandmarks> graph_;
  std::unique_ptr<OpenList> open_;
  CudaSASPlus *cuda_problem_;
  CudaSuccessorGenerator *cuda_generator_;
  CudaLandmarkGraph *cuda_landmark_graph_;
  CudaZobristHash *cuda_c_hash_;
  CudaZobristHash *cuda_d_hash_;
  CudaSearchGraph cuda_graph_;
  CudaOpenList cuda_open_;
  CudaClosedList cuda_closed_;
  GBFSMessage m_;
  GBFSMessage cuda_m_;
};

} // namespace pplaner

#endif // CUDA_HETRO_GBFS_H_
