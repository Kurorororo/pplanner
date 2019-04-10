#ifndef MCS_DUMP_H_
#define MCS_DUMP_H_

#include <atomic>
#include <iostream>
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
#include "hash/zobrist_hash.h"
#include "multithread_search/focus.h"
#include "multithread_search/lock_free_closed_list.h"
#include "open_list.h"
#include "open_list_factory.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph/state_packer.h"
#include "search_node.h"
#include "successor_generator.h"

namespace pplanner {

struct SearchNodeWithTimeStamp : public SearchNodeWithNext {
  long long int timestamp;
  int id;
};

class MCSDump : public Search {
 public:
  MCSDump(std::shared_ptr<const SASPlus> problem,
          const boost::property_tree::ptree &pt)
      : use_preferred_(false),
        n_threads_(1),
        min_expansion_per_focus_(1000),
        plateau_threshold_(10000),
        expanded_(0),
        evaluated_(0),
        generated_(0),
        dead_ends_(0),
        id_(0),
        problem_(problem),
        generator_(std::make_unique<SuccessorGenerator>(problem)),
        packer_(std::make_unique<StatePacker>(problem)),
        hash_(std::make_unique<ZobristHash>(problem, 4166245435)),
        foci_(PriorityQueueFactory<
              std::vector<int>,
              std::shared_ptr<Focus<SearchNodeWithTimeStamp *> > >("fifo")),
        start_(std::chrono::system_clock::now()) {
    Init(pt);
  }

  ~MCSDump();

  std::vector<int> Plan() override {
    auto goal = Search();

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

 private:
  SearchNodeWithTimeStamp *Search();

  void InitialEvaluate();

  void Expand(int i);

  int Evaluate(int i, const std::vector<int> &state,
               SearchNodeWithTimeStamp *node, std::vector<int> &values);

  int IncrementID();

  int IncrementNFoci();

  int DecrementNFoci();

  std::shared_ptr<Focus<SearchNodeWithTimeStamp *> > TryPopFocus(
      std::shared_ptr<Focus<SearchNodeWithTimeStamp *> > focus) {
    if (open_mtx_.try_lock()) {
      if (!foci_->IsEmpty() && foci_->MinimumValue() < focus->Priority()) {
        auto tmp_focus = foci_->Pop();
        foci_->Push(focus->Priority(), focus);
        focus = tmp_focus;
      }

      open_mtx_.unlock();
    }

    return focus;
  }

  std::shared_ptr<Focus<SearchNodeWithTimeStamp *> > LockedPopFocus() {
    std::lock_guard<std::mutex> lock(open_mtx_);

    if (foci_->IsEmpty()) return nullptr;

    return foci_->Pop();
  }

  void LockedPushFocus(
      std::shared_ptr<Focus<SearchNodeWithTimeStamp *> > focus) {
    std::lock_guard<std::mutex> lock(open_mtx_);

    foci_->Push(focus->Priority(), focus);
  }

  std::shared_ptr<Focus<SearchNodeWithTimeStamp *> > CreateNewFocus(
      const std::vector<int> &values, SearchNodeWithTimeStamp *node,
      bool is_pref) {
    return std::make_shared<Focus<SearchNodeWithTimeStamp *> >(
        open_list_option_, values, node, is_pref);
  }

  void WriteGoal(SearchNodeWithTimeStamp *goal) {
    SearchNodeWithTimeStamp *expected = nullptr;
    goal_.compare_exchange_strong(expected, goal);
  }

  void WriteStat(int expanded, int evaluated, int generated, int dead_ends) {
    std::lock_guard<std::mutex> lock(stat_mtx_);

    expanded_ += expanded;
    generated_ += generated;
    evaluated_ += evaluated;
    dead_ends_ += dead_ends;
  }

  void InitHeuristics(int i, const boost::property_tree::ptree pt);

  void Init(const boost::property_tree::ptree &pt);

  void DeleteAllNodes(int i);

  bool UpdateBestH(int h);

  bool use_preferred_;
  int n_threads_;
  int min_expansion_per_focus_;
  int n_foci_max_;
  int plateau_threshold_;
  int expanded_;
  int evaluated_;
  int generated_;
  int dead_ends_;
  std::atomic<int> n_foci_;
  std::atomic<int> id_;
  std::atomic<SearchNodeWithTimeStamp *> goal_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::unique_ptr<StatePacker> packer_;
  std::unique_ptr<ZobristHash> hash_;
  std::unique_ptr<LockFreeClosedList> closed_;
  std::vector<std::vector<SearchNodeWithTimeStamp *> > node_pool_;
  std::vector<std::shared_ptr<Evaluator> > preferring_;
  std::vector<std::vector<std::shared_ptr<Evaluator> > > evaluators_;
  boost::property_tree::ptree open_list_option_;
  std::unique_ptr<PriorityQueue<
      std::vector<int>, std::shared_ptr<Focus<SearchNodeWithTimeStamp *> > > >
      foci_;
  std::mutex open_mtx_;
  std::mutex stat_mtx_;
  std::chrono::system_clock::time_point start_;
};

}  // namespace pplanner

#endif  // MCS_DUMP_H_
