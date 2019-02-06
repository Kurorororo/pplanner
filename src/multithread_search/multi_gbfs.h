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

#include "evaluator.h"
#include "dominance/lds.h"
#include "open_list.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph.h"
#include "successor_generator.h"

namespace pplanner {

class MultiGBFS : public Search {
 public:
  MultiGBFS(std::shared_ptr<const SASPlus> problem,
            const boost::property_tree::ptree &pt)
    : use_preferred_(false),
      initialized_(false),
      n_threads_(1),
      expanded_(0),
      evaluated_(0),
      generated_(0),
      dead_ends_(0),
      goal_(-1),
      problem_(problem),
      preferring_(nullptr),
      generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
      graph_(nullptr),
      open_list_(nullptr) { Init(pt); }

  virtual ~MultiGBFS() {}

  std::vector<int> Plan() override {
    int goal = Search();
    auto plan = ExtractPath(graph_, goal);

    return plan;
  }

  void DumpStatistics() const override;

  virtual int Search();

  void InitialExpand();

  void  Expand();

  int Evaluate(const std::vector<int> &state, int node,
               std::vector<int> &values);

  int LockedPop() {
    std::lock_guard<std::mutex> lock(open_mtx_);

    if (open_list_->IsEmpty()) return -1;

    return open_list_->Pop();
  }

  void LockedPush(std::vector<int> &values, int node, bool is_preferred) {
    std::lock_guard<std::mutex> lock(open_mtx_);

    open_list_->Push(values, node, is_preferred);
  }

  void LockedClose(int node) {
    std::lock_guard<std::shared_timed_mutex> lock(closed_mtx_);
    graph_->Close(node);
  }

  bool LockedClosedCheck(uint32_t hash, const uint32_t* packed) {
    std::shared_lock<std::shared_timed_mutex> lock(closed_mtx_);

    return graph_->GetClosed(hash, packed) != -1;
  }

  int LockedAllocateNode() {
    std::lock_guard<std::mutex> lock(graph_mtx_);

    return graph_->AllocateNode();
  }

  bool LockedReadGoal() {
    std::shared_lock<std::shared_timed_mutex> lock(goal_mtx_);

    return goal_ != -1;
  }

  void LockedWriteGoal(int goal) {
    std::lock_guard<std::shared_timed_mutex> lock(goal_mtx_);
    goal_ = goal;
  }

  void WriteReporter() {
    std::lock_guard<std::mutex> lock(stat_mtx_);

    if (initialized_) return;

    initialized_ = true;
    reporter_tid_ = std::this_thread::get_id();
  }

  void WriteStat(int expanded, int evaluated, int generated, int dead_ends) {
    std::lock_guard<std::mutex> lock(stat_mtx_);
    expanded_ += expanded;
    generated_ += generated;
    evaluated_ += evaluated;
    dead_ends_ += dead_ends;
  }

 private:
  void Init(const boost::property_tree::ptree &pt);

  bool use_preferred_;
  bool initialized_;
  int n_threads_;
  int expanded_;
  int evaluated_;
  int generated_;
  int dead_ends_;
  int goal_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<Evaluator> preferring_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraph> graph_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
  std::unique_ptr<OpenList> open_list_;
  std::thread::id reporter_tid_;
  std::mutex graph_mtx_;
  std::mutex open_mtx_;
  std::mutex stat_mtx_;
  std::shared_timed_mutex closed_mtx_;
  std::shared_timed_mutex goal_mtx_;
};

} // namespace pplanner

#endif // MULTI_GBFS_H_
