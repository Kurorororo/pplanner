#ifndef CUDA_RWAGBFS_H_
#define CUDA_RWAGBFS_H_

#include <array>
#include <memory>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "cuda_sas_plus.cuh"
#include "cuda_successor_generator.cuh"
#include "open_list.h"
#include "sas_plus.h"
#include "search.h"
#include "search_graph/search_graph_with_landmarks.h"
#include "successor_generator.h"
#include "landmark/landmark_count_base.h"
#include "cuda_landmark/cuda_landmark_graph.cuh"
#include "cuda_search/cuda_random_walk.cuh"

namespace pplanner {

class CudaRWAGBFS : public Search {
 public:
  CudaRWAGBFS(std::shared_ptr<const SASPlus> problem,
           const boost::property_tree::ptree &pt)
    : n_grid_(20),
      n_block_(256),
      n_elite_(100),
      walk_length_(10),
      generated_(0),
      expanded_(0),
      evaluated_(0),
      dead_ends_(0),
      problem_(problem),
      generator_(std::make_shared<SuccessorGenerator>(problem)),
      graph_(nullptr),
      lmcount_(nullptr),
      open_list_(nullptr),
      cuda_problem_(new CudaSASPlus),
      cuda_generator_(new CudaSuccessorGenerator),
      cuda_landmark_graph_(new CudaLandmarkGraph) { Init(pt); }

  ~CudaRWAGBFS();

  std::vector<int> Plan() override {
    int goal = Search();

    return ExtractPlan(goal);
  }

  void DumpStatistics() const override;

 private:
  void Init(const boost::property_tree::ptree &pt);

  void GenerateChildren(int parent, const std::vector<int> &state);

  void InitialEvaluate();

  int PushStates(const std::vector<int> &parents, const RandomWalkMessage &m);

  int Search();

  int Expand(std::vector<int> &parents, int *counter, RandomWalkMessage *m, bool flag);

  std::vector<int> ExtractPlan(int node);

  int n_grid_;
  int n_block_;
  int n_threads_;
  int n_elite_;
  int walk_length_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int best_h_;
  int landmark_id_max_;
  int n_landmark_bytes_;
  std::vector<int> ranking;
  std::vector<std::vector<int> > sequences_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraphWithLandmarks> graph_;
  std::shared_ptr<LandmarkCountBase> lmcount_;
  std::unique_ptr<OpenList<int> > open_list_;
  CudaSASPlus *cuda_problem_;
  CudaSuccessorGenerator *cuda_generator_;
  CudaLandmarkGraph *cuda_landmark_graph_;
  std::array<RandomWalkMessage, 2> ms_;
  RandomWalkMessage cuda_m_;
};

} // namespace pplaner

#endif // CUDA_RWAGBFS_H_
