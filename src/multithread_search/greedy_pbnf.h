#ifndef GREEDY_PBNF_H_
#define GREEDY_PBNF_H_

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
#include "multithread_search/heuristic.h"
#include "multithread_search/nblock.h"
#include "multithread_search/search_node.h"
#include "sas_plus.h"
#include "search.h"
#include "search_graph/state_packer.h"
#include "successor_generator.h"

namespace pplanner {

class GreedyPBNF : public Search {
 public:
  GreedyPBNF(std::shared_ptr<const SASPlus> problem,
            const boost::property_tree::ptree &pt)
    : use_preferred_(false),
      n_threads_(1),
      max_abstract_nodes_(50000),
      expanded_(0),
      evaluated_(0),
      generated_(0),
      dead_ends_(0),
      problem_(problem),
      generator_(std::make_unique<SuccessorGenerator>(problem)),
      packer_(std::make_unique<StatePacker>(problem)),
      hash_(std::make_unique<ZobristHash>(problem, 4166245435)),
      abstract_graph_(nullptr){ Init(pt); }

  ~GreedyPBNF();

  std::vector<int> Plan() override {
    auto goal = Search();

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  void WriteGoal(SearchNodeWithNext* goal) {
    SearchNodeWithNext *expected = nullptr;
    goal_.compare_exchange_strong(expected, goal);
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

  int Evaluate(int i, const std::vector<int> &state, SearchNodeWithNext* node,
               std::vector<int> &values);

  void InitialEvaluate();

  void DeleteAllNodes(int i);

  const std::vector<int>& InferenceScope(std::shared_ptr<NBlock> b) const {
    return abstract_graph_->InterferenceScope(b->abstract_node_id());
  }

  void Sleep() {
    std::unique_lock<std::mutex> lock(sleep_mtx_);

    cond_.wait(lock, [this] { return !freelist_.empty() || done_; } );
  }

  void WakeAllSleepingThreads() { cond_.notify_all(); }

  std::shared_ptr<NBlock> BestScope(std::shared_ptr<NBlock> b) const;

  std::shared_ptr<NBlock> BestFree();

  SearchNodeWithNext* Search();

  void ThreadSearch(int i);

  bool ShouldSwitch(std::shared_ptr<NBlock> b, int &exp);

  void SetHot(std::shared_ptr<NBlock> b);

  void SetCold(std::shared_ptr<NBlock> b);

  void Release(std::shared_ptr<NBlock> b);

  std::shared_ptr<NBlock> NextNBlock(std::shared_ptr<NBlock> b);

  bool use_preferred_;
  int n_threads_;
  int n_abstract_nodes_;
  int expanded_;
  int evaluated_;
  int generated_;
  int dead_ends_;
  std::atomic<SearchNodeWithNext*> goal_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<StatePacker> packer_;
  std::unique_ptr<ZobristHash> hash_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::vector<std::shared_ptr<Heuristic<SearchNode*> > > preferring_;
  std::vector<std::vector<
    std::shared_ptr<Heuristic<SearchNode*> > > > evaluators_;
  std::shared_ptr<AbstractGraph> abstract_graph_;
  std::priority_queue<std::shared_ptr<NBlock> > freelist_;
  std::vector<std::shared_ptr<NBlock> > nblocks_;
  std::vector<std::vector<SearchNode*> > node_pool_;
  std::mutex mtx_;
  std::mutex sleep_mtx_;
  std::mutex stat_mtx_;
  std::condition_variable cond_;
};

} // namespace pplanner

#endif // GREEDY_PBNF_H_
