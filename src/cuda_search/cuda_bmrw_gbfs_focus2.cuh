#ifndef CUDA_BMRW_GBFS_FOCUS2_H_
#define CUDA_BMRW_GBFS_FOCUS2_H_

#include <array>
#include <memory>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "cuda_sas_plus.cuh"
#include "cuda_successor_generator.cuh"
#include "evaluator.h"
#include "open_list.h"
#include "sas_plus.h"
#include "search.h"
#include "search_graph/search_graph_with_landmarks.h"
#include "successor_generator.h"
#include "landmark/landmark_count_base.h"
#include "cuda_landmark/cuda_landmark_graph.cuh"
#include "cuda_search/cuda_random_walk.cuh"

namespace pplanner {

class CudaBMRWGBFSFocus2 : public Search {
 public:
  CudaBMRWGBFSFocus2(std::shared_ptr<const SASPlus> problem,
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
      rw_graph_(nullptr),
      lmcount_(nullptr),
      rw_open_(nullptr),
      cuda_problem_(new CudaSASPlus),
      cuda_generator_(new CudaSuccessorGenerator),
      cuda_landmark_graph_(new CudaLandmarkGraph) { Init(pt); }

  ~CudaBMRWGBFSFocus2();

  std::vector<int> Plan() override;

  void DumpStatistics() const override;

 private:
  void Init(const boost::property_tree::ptree &pt);

  void RWInitialEvaluate();

  void PopStates(std::vector<int> &parents);

  void GenerateChildren(int parent, std::vector<int> &values,
                        const std::vector<int> &state);

  int PushStates(const std::vector<int> &parents, std::vector<int> &arg_h);

  void Restart();

  int Search();

  void InitialEvaluate();

  int GreedyOpen();

  int CpuExpand();

  std::vector<int> ExtractPlanFromRWGraph(int node);

  std::vector<int> ExtractPlanFromGBFS(int node);

  std::vector<int> ExtractPlan(int node, bool from_rw);

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
  std::vector<std::vector<int> > sequences_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraphWithLandmarks> graph_;
  std::shared_ptr<SearchGraphWithLandmarks> rw_graph_;
  std::unordered_map<int, int> gbfs_node_to_rw_node_;
  std::unordered_map<int, int> rw_node_to_gbfs_node_;
  std::shared_ptr<LandmarkCountBase> lmcount_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
  boost::property_tree::ptree open_option_;
  std::vector<std::shared_ptr<OpenList<int> > > opens_;
  std::shared_ptr<OpenList<int> > rw_open_;
  CudaSASPlus *cuda_problem_;
  CudaSuccessorGenerator *cuda_generator_;
  CudaLandmarkGraph *cuda_landmark_graph_;
  RandomWalkMessage m_;
  RandomWalkMessage cuda_m_;
};

} // namespace pplaner

#endif // CUDA_BMRW_GBFS_FOCUS_2_H_
