#ifndef GREEDY_PBNF_H_
#define GREEDY_PBNF_H_

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "hash/zobrist_hash.h"
#include "multithread_search/nblock.h"
#include "sas_plus.h"
#include "sas_plus/abstract_graph.h"
#include "search.h"
#include "search_graph/state_packer.h"
#include "search_node.h"
#include "successor_generator.h"
#include "utils/binary_heap.h"

namespace pplanner {

class GreedyPBNF : public Search {
 public:
  GreedyPBNF(std::shared_ptr<const SASPlus> problem,
             const boost::property_tree::ptree &pt)
      : use_preferred_(false),
        done_(false),
        n_threads_(1),
        min_expansions_(1),
        expanded_(0),
        evaluated_(0),
        generated_(0),
        dead_ends_(0),
        problem_(problem),
        generator_(std::make_unique<SuccessorGenerator>(problem)),
        packer_(std::make_unique<StatePacker>(problem)),
        hash_(std::make_unique<ZobristHash>(problem, 4166245435)),
        abstract_graph_(nullptr) {
    Init(pt);
  }

  ~GreedyPBNF();

  std::vector<int> Plan() override {
    auto goal = Search();

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  void WriteGoal(SearchNode *goal) {
    SearchNode *expected = nullptr;
    goal_.compare_exchange_strong(expected, goal);
    done_ = true;
    cond_.notify_all();
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

  int Evaluate(int i, const std::vector<int> &state, SearchNode *node,
               std::vector<int> &values);

  void InitialEvaluate();

  void DeleteAllNodes(int i);

  const std::vector<int> &InterferenceScope(std::shared_ptr<NBlock> b) const {
    return abstract_graph_->InterferenceScope(b->abstract_node_id());
  }

  std::shared_ptr<NBlock> BestScope(std::shared_ptr<NBlock> b) const;

  SearchNode *Search();

  void ThreadSearch(int i);

  bool ShouldSwitch(int i, std::shared_ptr<NBlock> b, int *exp);

  void SetHot(std::shared_ptr<NBlock> b);

  bool SetCold(std::shared_ptr<NBlock> b);

  void Release(int i, std::shared_ptr<NBlock> b);

  std::shared_ptr<NBlock> NextNBlock(int i, std::shared_ptr<NBlock> b);

  struct NBlockLess {
    bool operator()(const std::shared_ptr<const NBlock> &a,
                    const std::shared_ptr<const NBlock> &b) const {
      if (a->IsEmpty()) return false;

      return b->IsEmpty() || a->MinimumValues() < b->MinimumValues();
    }
  };

  bool use_preferred_;
  bool done_;
  int n_threads_;
  int min_expansions_;
  int expanded_;
  int evaluated_;
  int generated_;
  int dead_ends_;
  std::atomic<SearchNode *> goal_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::unique_ptr<StatePacker> packer_;
  std::unique_ptr<ZobristHash> hash_;
  std::vector<std::shared_ptr<Evaluator> > preferring_;
  std::vector<std::vector<std::shared_ptr<Evaluator> > > evaluators_;
  std::shared_ptr<AbstractGraph> abstract_graph_;
  BinaryHeap<std::shared_ptr<NBlock> > freelist_;
  std::vector<std::shared_ptr<NBlock> > nblocks_;
  std::vector<std::vector<SearchNode *> > node_pool_;
  std::mutex mtx_;
  std::mutex stat_mtx_;
  std::condition_variable cond_;
};

}  // namespace pplanner

#endif  // GREEDY_PBNF_H_
