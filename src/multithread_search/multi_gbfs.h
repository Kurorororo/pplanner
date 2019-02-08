#ifndef MULTI_GBFS_H_
#define MULTI_GBFS_H_

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

#include "hash/zobrist_hash.h"
#include "multithread_search/closed_list.h"
#include "multithread_search/heuristic.h"
#include "multithread_search/search_node.h"
#include "open_list.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph/state_packer.h"
#include "successor_generator.h"

namespace pplanner {

class MultiGBFS : public Search {
 public:
  MultiGBFS(std::shared_ptr<const SASPlus> problem,
            const boost::property_tree::ptree &pt)
    : use_preferred_(false),
      n_threads_(1),
      expanded_(0),
      evaluated_(0),
      generated_(0),
      dead_ends_(0),
      goal_(nullptr),
      problem_(problem),
      generator_(std::make_unique<SuccessorGenerator>(problem)),
      packer_(std::make_unique<StatePacker>(problem)),
      hash1_(std::make_unique<ZobristHash>(problem, 4166245435)),
      hash2_(std::make_unique<ZobristHash>(problem, 2886379259)),
      open_list_(nullptr) { Init(pt); }

  ~MultiGBFS() {}

  std::vector<int> Plan() override {
    auto goal = Search();

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  std::shared_ptr<SearchNodeWithHash> Search();

  void InitialEvaluate();

  void Expand(int i);

  int Evaluate(int i, const std::vector<int> &state,
               std::shared_ptr<SearchNodeWithHash> node,
               std::vector<int> &values);

  std::shared_ptr<SearchNodeWithHash> LockedPop() {
    std::lock_guard<std::mutex> lock(open_mtx_);

    if (open_list_->IsEmpty()) return nullptr;

    return open_list_->Pop();
  }

  void LockedPush(std::vector<int> &values,
                  std::shared_ptr<SearchNodeWithHash> node,
                  bool is_preferred) {
    std::lock_guard<std::mutex> lock(open_mtx_);

    open_list_->Push(values, node, is_preferred);
  }

  void LockedClose(int i, std::shared_ptr<SearchNodeWithHash> node) {
    std::lock_guard<std::shared_timed_mutex> lock(*closed_mtx_[i]);

    closed_[i]->Close(node);
  }

  bool LockedIsClosed(int i, uint32_t hash,
                      const std::vector<uint32_t> packed) {
    std::shared_lock<std::shared_timed_mutex> lock(*closed_mtx_[i]);

    return closed_[i]->IsClosed(hash, packed);
  }

  bool LockedReadGoal() {
    std::shared_lock<std::shared_timed_mutex> lock(goal_mtx_);

    return goal_ != nullptr;
  }

  void LockedWriteGoal(std::shared_ptr<SearchNodeWithHash> goal) {
    std::lock_guard<std::shared_timed_mutex> lock(goal_mtx_);

    goal_ = goal;
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
  std::shared_ptr<SearchNodeWithHash> goal_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::unique_ptr<StatePacker> packer_;
  std::unique_ptr<ZobristHash> hash1_;
  std::unique_ptr<ZobristHash> hash2_;
  std::vector<std::shared_ptr<ClosedList> > closed_;
  std::vector<std::shared_ptr<Heuristic> > preferring_;
  std::vector<std::vector<std::shared_ptr<Heuristic> > > evaluators_;
  std::shared_ptr<OpenList<std::shared_ptr<SearchNodeWithHash> > > open_list_;
  std::mutex open_mtx_;
  std::mutex stat_mtx_;
  std::vector<std::unique_ptr<std::shared_timed_mutex> > closed_mtx_;
  std::shared_timed_mutex goal_mtx_;
};

} // namespace pplanner

#endif // MULTI_GBFS_H_
