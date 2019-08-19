#ifndef KPLG_H_
#define KPLG_H_

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

class KPLG : public Search {
 public:
  enum Status { OPEN, PENDING, WAITING, NO_SOLUTION };

  struct SearchNodeWithFlag : public SearchNodeWithNext {
    bool sure;
  };

  KPLG(std::shared_ptr<const SASPlus> problem,
       const boost::property_tree::ptree &pt)
      : use_preferred_(false),
        n_threads_(1),
        expanded_(0),
        evaluated_(0),
        generated_(0),
        dead_ends_(0),
        n_e_(0),
        n_sure_(0),
        n_p_(0),
        h_p_(-1),
        problem_(problem),
        generator_(std::make_unique<SuccessorGenerator>(problem)),
        packer_(std::make_unique<StatePacker>(problem)),
        hash_(std::make_unique<ZobristHash>(problem, 4166245435)),
        open_list_(nullptr),
        pending_list_(nullptr) {
    Init(pt);
  }

  ~KPLG() {}

  std::vector<int> Plan() override {
    auto goal = Search();

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  std::shared_ptr<SearchNodeWithNext> Search();

  void InitialEvaluate();

  void Expand(int i);

  int Evaluate(int i, const std::vector<int> &state,
               std::shared_ptr<SearchNodeWithFlag> node,
               std::vector<int> &values, const Status status);

  std::pair<std::shared_ptr<SearchNodeWithFlag>, Status> LockedPop();

  void LockedPush(std::vector<int> &values,
                  std::shared_ptr<SearchNodeWithFlag> node, bool is_preferred,
                  const Status status);

  void WriteGoal(std::shared_ptr<SearchNodeWithFlag> goal) {
    std::shared_ptr<SearchNodeWithFlag> expected = nullptr;
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
  std::atomic_int n_e_;
  std::atomic_int n_sure_;
  std::atomic_int n_p_;
  std::atomic_int h_p_;
  std::shared_ptr<SearchNodeWithFlag> goal_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::unique_ptr<StatePacker> packer_;
  std::unique_ptr<ZobristHash> hash_;
  std::unique_ptr<LockFreeClosedList> closed_;
  std::vector<std::shared_ptr<Evaluator> > preferring_;
  std::vector<std::vector<std::shared_ptr<Evaluator> > > evaluators_;
  std::shared_ptr<
      OpenList<std::vector<int>, std::shared_ptr<SearchNodeWithFlag> > >
      open_list_;
  std::shared_ptr<
      OpenList<std::vector<int>, std::shared_ptr<SearchNodeWithFlag> > >
      pending_list_;
  std::mutex open_mtx_;
  std::mutex pending_mtx_;
  std::mutex stat_mtx_;
};

}  // namespace pplanner

#endif  // KPLG_H_
