#ifndef PGBFS_H_
#define PGBFS_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <random>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "closed_list.h"
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

class PGBFS : public Search {
 public:
  PGBFS(std::shared_ptr<const SASPlus> problem,
        const boost::property_tree::ptree &pt)
      : use_preferred_(false),
        n_threads_(1),
        expanded_(0),
        evaluated_(0),
        generated_(0),
        dead_ends_(0),
        n_cached_(0),
        problem_(problem),
        generator_(std::make_unique<SuccessorGenerator>(problem)),
        packer_(std::make_unique<StatePacker>(problem)),
        hash_(std::make_unique<ZobristHash>(problem, 4166245435)),
        cached_(nullptr),
        shared_closed_(nullptr) {
    Init(pt);
  }

  ~PGBFS() {}

  std::vector<int> Plan() override {
    auto goal = Search();

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  std::shared_ptr<SearchNode> Search();

  void InitialEvaluate();

  void Expand(int i);

  void WriteGoal(std::shared_ptr<SearchNode> goal) {
    std::shared_ptr<SearchNode> expected = nullptr;
    std::atomic_compare_exchange_strong(&goal_, &expected, goal);
  }

  void WriteStat(int expanded, int evaluated, int generated, int dead_ends,
                 int n_cached) {
    std::lock_guard<std::mutex> lock(stat_mtx_);

    expanded_ += expanded;
    generated_ += generated;
    evaluated_ += evaluated;
    dead_ends_ += dead_ends;
    n_cached_ += n_cached;
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
  int n_cached_;
  std::shared_ptr<SearchNode> goal_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::unique_ptr<StatePacker> packer_;
  std::unique_ptr<ZobristHash> hash_;
  std::vector<std::shared_ptr<ClosedList>> closed_lists_;
  std::unique_ptr<LockFreeClosedList<SearchNodeWithNext>> cached_;
  std::unique_ptr<LockFreeClosedList<SearchNodeWithNext>> shared_closed_;
  std::vector<std::shared_ptr<Evaluator>> preferring_;
  std::vector<std::shared_ptr<Evaluator>> evaluators_;
  std::vector<
      std::shared_ptr<OpenList<int, std::shared_ptr<SearchNodeWithNext>>>>
      open_lists_;
  std::mutex stat_mtx_;
};

}  // namespace pplanner

#endif  // PGBFS_H_
