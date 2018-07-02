#ifndef HDGBFS_H_
#define HDGBFS_H_

#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <mpi.h>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "sas_plus.h"
#include "search.h"
#include "search_graph/distributed_search_graph.h"
#include "successor_generator.h"
#include "open_list.h"
#include "hash/zobrist_hash.h"

namespace pplanner {

class HDGBFS : public Search {
 public:
  HDGBFS(std::shared_ptr<const SASPlus> problem,
         const boost::property_tree::ptree &pt)
    : use_preferred_(false),
      generated_(0),
      expanded_(0),
      evaluated_(0),
      dead_ends_(0),
      n_preferred_evaluated_(0),
      n_branching_(0),
      n_preferreds_(0),
      best_h_(-1),
      world_size_(2),
      rank_(0),
      mpi_buffer_(nullptr),
      tmp_state_(problem->n_variables()),
      problem_(problem),
      preferring_(nullptr),
      generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
      graph_(nullptr),
      open_list_(nullptr),
      z_hash_(std::unique_ptr<ZobristHash>(new ZobristHash(problem))) {
    Init(pt);
  }

  virtual ~HDGBFS() {
    Flush();
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

  virtual int Expand(int node, std::vector<int> &state, std::vector<int> &child,
                     std::vector<int> &applicable,
                     std::unordered_set<int> &preferred);

  bool NoNode() const { return open_list_->IsEmpty(); }

  int NodeToExpand() { return open_list_->Pop(); }

  std::vector<int> InitialExpand();

  void SendNodes();

  void ReceiveNodes();

  void SendTermination();

  bool ReceiveTermination();

  static constexpr int kNodeTag = 0;
  static constexpr int kTerminationTag = 1;
  static constexpr int kPlanTag = 2;
  static constexpr int kPlanTerminationTag = 3;

 private:
  void Init(const boost::property_tree::ptree &pt);

  void Evaluate(const std::vector<int> &state, int node);

  std::vector<int> ExtractPath(int node);

  void Flush();

  bool use_preferred_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int n_preferred_evaluated_;
  int n_branching_;
  int n_preferreds_;
  int best_h_;
  int world_size_;
  int rank_;
  unsigned char *mpi_buffer_;
  std::vector<int> tmp_state_;
  std::vector<unsigned char> incoming_buffer_;
  std::vector<std::vector<unsigned char> > outgoing_buffers_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<Evaluator> preferring_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<DistributedSearchGraph> graph_;
  std::unique_ptr<OpenList> open_list_;
  std::unique_ptr<ZobristHash> z_hash_;
};

} // namespace pplanner

#endif // HDGBFS_H_
