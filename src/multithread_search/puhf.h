#ifndef PUHF_H_
#define PUHF_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "hash/zobrist_hash.h"
#include "multithread_search/lock_free_closed_list.h"
#include "open_list.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph/state_packer.h"
#include "search_node.h"
#include "successor_generator.h"

namespace pplanner {

class PUHF : public Search {
 public:
  PUHF(std::shared_ptr<const SASPlus> problem,
       const boost::property_tree::ptree &pt)
      : use_preferred_(false),
        n_threads_(1),
        expanded_(0),
        evaluated_(0),
        generated_(0),
        dead_ends_(0),
        n_expanding_(0),
        h_expanding_(-1),
        problem_(problem),
        generator_(std::make_unique<SuccessorGenerator>(problem)),
        packer_(std::make_unique<StatePacker>(problem)),
        hash_(std::make_unique<ZobristHash>(problem, 4166245435)),
        open_list_(nullptr),
        pending_list_(nullptr) {
    Init(pt);
  }

  ~PUHF() {}

  std::vector<int> Plan() override {
    auto goal = Search();

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  std::shared_ptr<SearchNodeWithNext> Search();

  void InitialEvaluate();

  void Expand(int i);

  int Evaluate(int i, const std::vector<int> &state,
               std::shared_ptr<SearchNodeWithNext> node,
               std::vector<int> &values);

  int EvaluatePending(int i, const std::vector<int> &state,
                      std::shared_ptr<SearchNodeWithNext> node,
                      std::vector<int> &values);

  std::pair<std::shared_ptr<SearchNodeWithNext>, bool> LockedPop();

  void LockedPush(int n, const std::vector<std::vector<int> > &values_buffer,
                  std::vector<std::shared_ptr<SearchNodeWithNext> > node_buffer,
                  std::vector<bool> is_preferred_buffer);

  void LockedPushPending(
      int n, const std::vector<std::vector<int> > &values_buffer,
      std::vector<std::shared_ptr<SearchNodeWithNext> > node_buffer,
      std::vector<bool> is_preferred_buffer);

  void WriteGoal(std::shared_ptr<SearchNodeWithNext> goal) {
    std::shared_ptr<SearchNodeWithNext> expected = nullptr;
    std::atomic_compare_exchange_strong(&goal_, &expected, goal);
  }

  void WriteStat(int expanded, int evaluated, int generated, int dead_ends) {
    std::lock_guard<std::mutex> lock(stat_mtx_);

    expanded_ += expanded;
    generated_ += generated;
    evaluated_ += evaluated;
    dead_ends_ += dead_ends;
  }

 private:
  void InitHeuristics(int i, const boost::property_tree::ptree pt);

  void Init(const boost::property_tree::ptree &pt);

  bool use_preferred_;
  int n_threads_;
  int expanded_;
  int evaluated_;
  int generated_;
  int dead_ends_;
  std::atomic_int n_expanding_;
  std::atomic_int h_expanding_;
  std::shared_ptr<SearchNodeWithNext> goal_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::unique_ptr<StatePacker> packer_;
  std::unique_ptr<ZobristHash> hash_;
  std::unique_ptr<LockFreeClosedList> closed_;
  std::vector<std::shared_ptr<Evaluator> > preferring_;
  std::vector<std::vector<std::shared_ptr<Evaluator> > > evaluators_;
  std::shared_ptr<
      OpenList<std::vector<int>, std::shared_ptr<SearchNodeWithNext> > >
      open_list_;
  std::shared_ptr<
      OpenList<std::vector<int>, std::shared_ptr<SearchNodeWithNext> > >
      pending_list_;
  std::mutex open_mtx_;
  std::mutex pending_mtx_;
  std::mutex stat_mtx_;
};

}  // namespace pplanner

#endif  // PUHF_H_