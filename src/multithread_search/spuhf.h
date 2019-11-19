#ifndef SPUHF_H_
#define SPUHF_H_

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

class SPUHF : public Search {
 public:
  struct SearchNodeWithFlag : public SearchNode {
    std::shared_ptr<SearchNodeWithFlag> next;
    bool certain;
  };

  SPUHF(std::shared_ptr<const SASPlus> problem,
        const boost::property_tree::ptree &pt)
      : use_preferred_(false),
        speculative_(false),
        goal_from_speculation_(false),
        dump_(false),
        n_threads_(1),
        expanded_(0),
        evaluated_(0),
        generated_(0),
        dead_ends_(0),
        n_cached_(0),
        n_expanding_(0),
        h_expanding_(-1),
        problem_(problem),
        generator_(std::make_unique<SuccessorGenerator>(problem)),
        packer_(std::make_unique<StatePacker>(problem)),
        hash_(std::make_unique<ZobristHash>(problem, 4166245435)),
        open_list_(nullptr) {
    Init(pt);
  }

  ~SPUHF() {}

  std::vector<int> Plan() override {
    auto goal = Search();

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  std::shared_ptr<SearchNodeWithFlag> Search();

  void InitialEvaluate();

  void Expand(int i);

  std::pair<bool, std::shared_ptr<SearchNodeWithFlag>> LockedPop();

  std::shared_ptr<SearchNodeWithFlag> SpeculativePop();

  void LockedPush(std::vector<std::shared_ptr<SearchNodeWithFlag>> node_buffer,
                  std::vector<bool> is_preferred_buffer);

  void SpeculativePush(bool from_open, std::vector<std::shared_ptr<SearchNodeWithFlag>> node_buffer, std::vector<bool> is_preferred_buffer);

  void WriteGoal(std::shared_ptr<SearchNodeWithFlag> goal) {
    std::shared_ptr<SearchNodeWithFlag> expected = nullptr;
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
  bool speculative_;
  bool goal_from_speculation_;
  bool dump_;
  int n_threads_;
  int expanded_;
  int evaluated_;
  int generated_;
  int dead_ends_;
  int n_cached_;
  std::atomic_int n_expanding_;
  std::atomic_int h_expanding_;
  std::shared_ptr<SearchNodeWithFlag> goal_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::unique_ptr<StatePacker> packer_;
  std::unique_ptr<ZobristHash> hash_;
  std::unique_ptr<LockFreeClosedList<SearchNodeWithFlag>> closed_;
  std::unique_ptr<LockFreeClosedList<SearchNodeWithFlag>> cached_;
  std::vector<std::shared_ptr<Evaluator>> preferring_;
  std::vector<std::shared_ptr<Evaluator>> evaluators_;
  std::shared_ptr<OpenList<int, std::shared_ptr<SearchNodeWithFlag>>>
      open_list_;
  std::shared_ptr<
      OpenList<std::pair<int, int>, std::shared_ptr<SearchNodeWithFlag>>>
      speculative_list_;
  std::vector<std::shared_ptr<SearchNode>> expanded_nodes_;
  std::mutex open_mtx_;
  std::mutex speculative_mtx_;
  std::mutex stat_mtx_;
};

}  // namespace pplanner

#endif  // SPUHF_H_
