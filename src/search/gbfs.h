#ifndef GBFS_H_
#define GBFS_H_

#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "dominance/lds.h"
#include "evaluator.h"
#include "open_list.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph.h"
#include "successor_generator.h"

namespace pplanner {

class GBFS : public Search {
 public:
  GBFS(std::shared_ptr<const SASPlus> problem,
       const boost::property_tree::ptree &pt)
      : use_preferred_(false),
        exhaust_(false),
        limit_expansion_(false),
        use_sss_(false),
        use_dominance_(false),
        sss_checked_(false),
        max_expansion_(0),
        generated_(0),
        expanded_(0),
        evaluated_(0),
        dead_ends_(0),
        n_preferred_evaluated_(0),
        n_branching_(0),
        n_preferreds_(0),
        n_pruned_(0),
        n_pruning_disable_(1000),
        n_plan_step_(-1),
        n_d_pruned_(0),
        min_pruning_ratio_(0.0),
        problem_(problem),
        preferring_(nullptr),
        generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
        graph_(nullptr),
        open_list_(nullptr),
        sss_aproximater_(nullptr),
        lds_(nullptr) {
    Init(pt);
  }

  virtual ~GBFS() {}

  std::vector<int> Plan() override {
    int goal = Search();
    auto plan = ExtractPath(graph_, goal);
    n_plan_step_ = plan.size();

    return plan;
  }

  void DumpStatistics() const override;

  virtual int Search();

  bool NoNode() const { return open_list_->IsEmpty(); }

  int NodeToExpand();

  std::vector<int> InitialExpand();

  int Expand(int node, std::vector<int> &state, std::vector<int> &child,
             std::vector<int> &applicable, std::unordered_set<int> &preferred);

  bool IsLimit() const {
    return limit_expansion_ && expanded_ > max_expansion_;
  }

 private:
  void Init(const boost::property_tree::ptree &pt);

  bool use_preferred_;
  bool exhaust_;
  bool limit_expansion_;
  bool use_sss_;
  bool use_dominance_;
  bool sss_checked_;
  int max_expansion_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int n_preferred_evaluated_;
  int n_branching_;
  int n_preferreds_;
  int best_h_;
  int n_pruned_;
  int n_pruning_disable_;
  int n_plan_step_;
  int n_d_pruned_;
  double min_pruning_ratio_;
  std::shared_ptr<const SASPlus> problem_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
  std::shared_ptr<Evaluator> preferring_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraph> graph_;
  std::unique_ptr<OpenList<> > open_list_;
  std::unique_ptr<SSSApproximater> sss_aproximater_;
  std::unique_ptr<LDS> lds_;
};

}  // namespace pplanner

#endif  // GBFS_H_
