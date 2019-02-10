#ifndef MULTI_FOCUS_GBFS_H_
#define MULTI_FOCUS_GBFS_H_

#include <memory>
#include <mutex>
#include <random>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>
#include <iostream>

#include <boost/property_tree/ptree.hpp>

#include "hash/zobrist_hash.h"
#include "multithread_search/closed_list.h"
#include "multithread_search/heuristic.h"
#include "multithread_search/search_node.h"
#include "open_list.h"
#include "open_list_factory.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph/state_packer.h"
#include "successor_generator.h"

namespace pplanner {

class Focus {
 public:
  Focus() : best_h_(-1), open_list_(nullptr) {}

  Focus(const boost::property_tree::ptree &pt, const std::vector<int> &values,
        std::shared_ptr<SearchNodeWithHash> node, bool is_pref,
        int plateau_threshold)
    : best_h_(values[0]),
      open_list_(OpenListFactory<std::shared_ptr<SearchNodeWithHash> >(pt)) {
    open_list_->Push(values, node, is_pref);
  }

  int best_h() const { return best_h_; }

  void set_best_h(int h) { best_h_ = h; }

  bool IsEmpty() const { return open_list_->IsEmpty(); }

  void Push(const std::vector<int> &values,
            std::shared_ptr<SearchNodeWithHash> node, bool is_pref) {
    open_list_->Push(values, node, is_pref);
  }

  std::shared_ptr<SearchNodeWithHash> Pop() { return open_list_->Pop(); }

  const std::vector<int> MinimumValues() const {
    return open_list_->MinimumValues();
  }

  void Boost() { open_list_->Boost(); }

 private:
  int best_h_;
  std::unique_ptr<OpenList<std::shared_ptr<SearchNodeWithHash> > > open_list_;
};

class MultiFocusGBFS : public Search {
 public:
  MultiFocusGBFS(std::shared_ptr<const SASPlus> problem,
                 const boost::property_tree::ptree &pt)
    : use_preferred_(false),
      n_threads_(1),
      min_expansion_per_focus_(1),
      plateau_threshold_(10000),
      expanded_(0),
      evaluated_(0),
      generated_(0),
      dead_ends_(0),
      goal_(nullptr),
      problem_(problem),
      generator_(std::make_unique<SuccessorGenerator>(problem)),
      packer_(std::make_unique<StatePacker>(problem)),
      hash1_(std::make_unique<ZobristHash>(problem, 4166245435)),
      hash2_(std::make_unique<ZobristHash>(problem, 2886379259)) { Init(pt); }

  ~MultiFocusGBFS() {}

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

  std::shared_ptr<Focus> LockedPopFocus() {
    std::lock_guard<std::mutex> lock(open_mtx_);

    if (foci_->IsEmpty()) return nullptr;

    return foci_->Pop();
  }

  void LockedPushFocus(std::shared_ptr<Focus> focus) {
    std::lock_guard<std::mutex> lock(open_mtx_);

    foci_->Push(focus->MinimumValues(), focus, false);
  }

  void CreateNewFocus(std::vector<int> &values,
                      std::shared_ptr<SearchNodeWithHash> node, bool is_pref) {
    auto focus = std::make_shared<Focus>(open_list_option_, values, node,
                                         is_pref, plateau_threshold_);
    LockedPushFocus(focus);
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
  int min_expansion_per_focus_;
  int plateau_threshold_;
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
  boost::property_tree::ptree open_list_option_;
  std::unique_ptr<OpenList<std::shared_ptr<Focus> > > foci_;
  std::mutex open_mtx_;
  std::mutex stat_mtx_;
  std::vector<std::unique_ptr<std::shared_timed_mutex> > closed_mtx_;
  std::shared_timed_mutex goal_mtx_;
};

} // namespace pplanner

#endif // MULTI_FOCUS_GBFS_H_
