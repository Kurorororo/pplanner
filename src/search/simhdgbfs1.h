#ifndef SIMHDGBFS1_H_
#define SIMHDGBFS1_H_

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "dominance/lds.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph/distributed_search_graph.h"
#include "successor_generator.h"
#include "open_list.h"
#include "hash/distribution_hash.h"

namespace pplanner {

class SIMHDGBFS1 : public Search {
 public:
  SIMHDGBFS1(std::shared_ptr<const SASPlus> problem,
         const boost::property_tree::ptree &pt)
    : limit_expansion_(false),
      use_local_open_(false),
      take_(0),
      max_expansion_(0),
      generated_(0),
      expanded_(0),
      evaluated_(0),
      dead_ends_(0),
      n_branching_(0),
      n_sent_(0),
      n_sent_or_generated_(0),
      best_h_(-1),
      world_size_(1),
      rank_(0),
      n_pushed_next_(0),
      n_sent_next_(0),
      delay_(1),
      delay_index_(0),
      n_evaluators_(0),
      problem_(problem),
      generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
      z_hash_(nullptr) { Init(pt); }

  virtual ~SIMHDGBFS1() {}

  std::vector<int> Plan() override {
    int goal = Search();

    return ExtractPath(goal);
  }

  void DumpStatistics() const override;

  virtual int Search();

  bool limit_expansion() const { return limit_expansion_; }

  int max_expansion() const { return max_expansion_; }

  int expanded() const { return expanded_; }

  int best_h() const { return best_h_; }

  void set_best_h(int h) { best_h_ = h; }

  int world_size() const { return world_size_; }

  size_t n_evaluators() const { return n_evaluators_; }

  int rank() const { return rank_; }

  size_t node_size() const { return graphs_[rank_]->node_size(); }

  std::shared_ptr<const SASPlus> problem() const { return problem_; }

  std::vector<int> InitialEvaluate();

  int Expand(int node, std::vector<int> &state);

  int Evaluate(const std::vector<int> &state, int node, int parent,
               std::vector<int> &values);

  void Push(std::vector<int> &values, int node, bool is_local);

  int Pop() {
    if (use_local_open_ && !local_open_lists_[rank_]->IsEmpty())
      return local_open_lists_[rank_]->Pop();

    return open_lists_[rank_]->Pop();
  }

  bool NoNode() const {
    return open_lists_[rank_]->IsEmpty()
      && (!use_local_open_ || local_open_lists_[rank_]->IsEmpty());
  }

  const std::vector<int>& MinimumValues() const {
    if (use_local_open_ && !local_open_lists_[rank_]->IsEmpty()
        && (open_lists_[rank_]->IsEmpty()
          || local_open_lists_[rank_]->MinimumValues()
          < open_lists_[rank_]->MinimumValues()))
      return local_open_lists_[rank_]->MinimumValues();

    return open_lists_[rank_]->MinimumValues();
  }

  unsigned char* ExtendOutgoingBuffer(int i, size_t size) {
    size_t index = outgoing_buffers_[rank_][i].size();
    outgoing_buffers_[rank_][i].resize(index + size);

    return outgoing_buffers_[rank_][i].data() + index;
  }

  void CommunicateNodes();

  void IncrementGenerated() { ++generated_; }

  void IncrementDeadEnds() { ++dead_ends_; }

 private:
  void Init(const boost::property_tree::ptree &pt);

  std::vector<int> ExtractPath(int node);

  bool limit_expansion_;
  bool use_local_open_;
  // 0: better 1: best 2: none
  int take_;
  int max_expansion_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int n_branching_;
  int n_sent_;
  int n_sent_or_generated_;
  int best_h_;
  int world_size_;
  int rank_;
  int n_pushed_next_;
  int n_sent_next_;
  int delay_;
  int delay_index_;
  size_t n_evaluators_;
  std::vector<std::vector<int> > best_values_;
  std::vector<std::vector<std::vector<unsigned char> > > incoming_buffers_;
  std::vector<std::vector<std::vector<unsigned char> > > outgoing_buffers_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<DistributionHash> z_hash_;
  std::vector<std::vector<std::shared_ptr<Evaluator> > > evaluators_;
  std::vector<std::shared_ptr<DistributedSearchGraph> > graphs_;
  std::vector<std::unique_ptr<OpenList> > open_lists_;
  std::vector<std::unique_ptr<OpenList> > local_open_lists_;
};

} // namespace pplanner

#endif // SIMHDGBFS1_H_
