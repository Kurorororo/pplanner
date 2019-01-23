#ifndef HETRO_HDGBFS_LG_H_
#define HETRO_HDGBFS_LG_H_

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <mpi.h>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "dominance/lds.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph/distributed_search_graph.h"
#include "successor_generator.h"
#include "open_list.h"
#include "hash/distribution_hash.h"

namespace pplanner {

class HetroHDGBFSLG : public Search {
 public:
  HetroHDGBFSLG(std::shared_ptr<const SASPlus> problem,
         const boost::property_tree::ptree &pt)
    : generated_(0),
      expanded_(0),
      expanded_local_(0),
      n_sent_(0),
      n_sent_or_generated_(0),
      n_received_(0),
      best_h_(-1),
      best_node_(-1),
      local_best_h_(-1),
      local_node_(-1),
      initial_rank_(0),
      world_size_(1),
      rank_(0),
      mpi_buffer_(nullptr),
      problem_(problem),
      generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
      lmcount_(std::make_shared<LandmarkCountBase>(
            problem, false, false, false, false)),
      d_hash_(std::make_shared<ZobristHash>(problem, 2886379259)),
      graph_(std::make_shared<DistributedSearchGraphWithLandmarks>(
            problem, 22)),
      open_(nullptr),
      local_node_(-1),
      cuda_problem_(new CudaSASPlus),
      cuda_generator_(new CudaSuccessorGenerator),
      cuda_landmark_graph_(new CudaLandmarkGraph),
      cuda_c_hash_(new CudaZobristHash),
      cuda_d_hash_(new CudaZobristHash) { Init(pt); }

  ~HetroHDGBFSLG();

  std::vector<int> Plan() override {
    MPI_Barrier(MPI_COMM_WORLD);
    int goal = Search();
    MPI_Barrier(MPI_COMM_WORLD);

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  virtual int Search();

  std::vector<int> InitialEvaluate();

  int Expand(int node, std::vector<int> &state);

  int Evaluate(const std::vector<int> &state, int node, int parent,
               std::vector<int> &values);

  void Push(std::vector<int> &values, int node, bool is_local);

  int Pop() {
    if (local_node_ == -1) return open_list_->Pop();

    int node = local_node_;
    local_node_ = -1;

    return node;
  }

  bool NoNode() const { return open_list_->IsEmpty() && local_node_ == -1; }

  bool LocalNoNode() const { return local_node_ == -1; }


  unsigned char* IncomingBuffer() { return incoming_buffer_.data(); }

  void ResizeIncomingBuffer(std::size_t size) {
    incoming_buffer_.resize(size);
  }

  unsigned char* ExtendOutgoingBuffer(int i, std::size_t size) {
    std::size_t index = outgoing_buffers_[i].size();
    outgoing_buffers_[i].resize(index + size);

    return outgoing_buffers_[i].data() + index;
  }

  void ClearOutgoingBuffer(int i) { outgoing_buffers_[i].clear(); }

  void SendNodes(int tag);

  void ReceiveNodes();

  void SendTermination();

  bool ReceiveTermination();

  void Flush(int tag);

  static constexpr int kNodeTag = 0;
  static constexpr int kTerminationTag = 1;
  static constexpr int kPlanTag = 2;
  static constexpr int kPlanTerminationTag = 3;
  static constexpr int kNodeWithSequenceTag = 4;

 private:
  void Init(const boost::property_tree::ptree &pt);

  std::vector<int> ExtractPath(int node);

  int generated_;
  int expanded_;
  int expanded_local_;
  int n_sent_;
  int n_sent_or_generated_;
  int n_received_;
  int best_h_;
  int initial_rank_;
  int world_size_;
  int rank_;
  unsigned char *mpi_buffer_;
  std::vector<int> best_values_;
  std::vector<unsigned char> incoming_buffer_;
  std::vector<std::vector<unsigned char> > outgoing_buffers_;
  std::unordered_map<int, std::vector<int> > sequences_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<LandmarkCountBase> lmcount_;
  std::shared_ptr<ZobristHash> d_hash_;
  std::shared_ptr<DistributedSearchGraphWithLandmarks> graph_;
  std::unique_ptr<OpenList> open_list_;
  std::unique_ptr<OpenList> local_open_list_;
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
};

} // namespace pplanner

#endif // HETRO_HDGBFS_LG_H_
