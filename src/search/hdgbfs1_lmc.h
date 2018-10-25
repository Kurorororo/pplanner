#ifndef HDGBFS1_LMC_H_
#define HDGBFS1_LMC_H_

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <mpi.h>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph/distributed_search_graph.h"
#include "successor_generator.h"
#include "open_list.h"
#include "hash/distribution_hash.h"

namespace pplanner {

class HDGBFS1Lmc : public Search {
 public:
  HDGBFS1Lmc(std::shared_ptr<const SASPlus> problem,
         const boost::property_tree::ptree &pt)
    : use_preferred_(false),
      limit_expansion_(false),
      use_sss_(false),
      take_best_(false),
      max_expansion_(0),
      generated_(0),
      expanded_(0),
      evaluated_(0),
      dead_ends_(0),
      n_preferred_evaluated_(0),
      n_branching_(0),
      n_preferreds_(0),
      n_sent_(0),
      n_sent_or_generated_(0),
      n_received_(0),
      best_h_(-1),
      initial_rank_(0),
      world_size_(1),
      rank_(0),
      n_evaluators_(0),
      mpi_buffer_(nullptr),
      problem_(problem),
      preferring_(nullptr),
      generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
      graph_(nullptr),
      open_list_(nullptr),
      z_hash_(nullptr),
      sss_aproximater_(nullptr) { Init(pt); }

  virtual ~HDGBFS1Lmc() {
    Flush(kNodeTag);
    int detach_size;
    MPI_Buffer_detach(&mpi_buffer_, &detach_size);
    delete[] mpi_buffer_;
  }

  std::vector<int> Plan() override {
    MPI_Barrier(MPI_COMM_WORLD);
    int goal = Search();
    MPI_Barrier(MPI_COMM_WORLD);

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  virtual int Search();

  bool limit_expansion() const { return limit_expansion_; }

  int max_expansion() const { return max_expansion_; }

  int expanded() const { return expanded_; }

  int best_h() const { return best_h_; }

  void set_best_h(int h) { best_h_ = h; }

  int initial_rank() const { return initial_rank_; }

  int world_size() const { return world_size_; }

  size_t n_evaluators() const { return n_evaluators_; }

  int rank() const { return rank_; }

  size_t node_size() const { return graph_->node_size(); }

  std::shared_ptr<const SASPlus> problem() const { return problem_; }

  std::shared_ptr<DistributedSearchGraph> graph() { return graph_; }

  std::vector<int> InitialEvaluate();

  int Expand(int node, std::vector<int> &state);

  int Evaluate(const std::vector<int> &state, int node,
               std::vector<int> &values);

  void Push(std::vector<int> &values, int node);

  int Pop() { return open_list_->Pop(); }

  bool NoNode() const { return open_list_->IsEmpty(); }

  const std::vector<int>& MinimumValues() const {
    return open_list_->MinimumValues();
  }

  size_t n_open_nodes() const { return open_list_->size(); }

  unsigned char* IncomingBuffer() { return incoming_buffer_.data(); }

  void ResizeIncomingBuffer(size_t size) { incoming_buffer_.resize(size); }

  unsigned char* ExtendOutgoingBuffer(int i, size_t size) {
    size_t index = outgoing_buffers_[i].size();
    outgoing_buffers_[i].resize(index + size);

    return outgoing_buffers_[i].data() + index;
  }

  void ClearOutgoingBuffer(int i) { outgoing_buffers_[i].clear(); }

  void SendNodes(int tag);

  void ReceiveNodes();

  void SendTermination();

  bool ReceiveTermination();

  void IncrementGenerated() { ++generated_; }

  void IncrementDeadEnds() { ++dead_ends_; }

  void Flush(int tag);

  static constexpr int kNodeTag = 0;
  static constexpr int kTerminationTag = 1;
  static constexpr int kPlanTag = 2;
  static constexpr int kPlanTerminationTag = 3;

 private:
  void Init(const boost::property_tree::ptree &pt);

  std::vector<int> ExtractPath(int node);

  bool use_preferred_;
  bool limit_expansion_;
  bool use_sss_;
  bool take_best_;
  int max_expansion_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int n_preferred_evaluated_;
  int n_branching_;
  int n_preferreds_;
  int n_sent_;
  int n_sent_or_generated_;
  int n_received_;
  int best_h_;
  int initial_rank_;
  int world_size_;
  int rank_;
  size_t n_evaluators_;
  unsigned char *mpi_buffer_;
  std::vector<int> best_values_;
  std::vector<unsigned char> incoming_buffer_;
  std::vector<std::vector<unsigned char> > outgoing_buffers_;
  std::shared_ptr<const SASPlus> problem_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
  std::shared_ptr<Evaluator> preferring_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<DistributedSearchGraphWithLandmarks> graph_;
  std::unique_ptr<OpenList> open_list_;
  std::shared_ptr<DistributionHash> z_hash_;
  std::unique_ptr<SSSApproximater> sss_aproximater_;
};

} // namespace pplanner

#endif // HDGBFS1_LMC_H_
