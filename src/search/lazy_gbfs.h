#ifndef LAZY_GBFS_H_
#define LAZY_GBFS_H_

#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "open_list.h"
#include "sas_plus.h"
#include "search.h"
#include "search_graph.h"
#include "successor_generator.h"

namespace pplanner {

class LazyGBFS : public Search {
 public:
  LazyGBFS(std::shared_ptr<const SASPlus> problem,
           const boost::property_tree::ptree &pt)
      : use_preferred_(false),
        same_(false),
        generated_(0),
        expanded_(0),
        evaluated_(0),
        dead_ends_(0),
        n_branching_(0),
        n_preferreds_(0),
        is_preferred_action_(problem->n_actions(), false),
        problem_(problem),
        preferring_(nullptr),
        generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
        graph_(nullptr),
        open_list_(nullptr) {
    Init(pt);
  }

  ~LazyGBFS() {}

  std::vector<int> Plan() override {
    int goal = Search();
    plan_ = ExtractPath(graph_, goal);

    return plan_;
  }

  void DumpStatistics() const override;

 private:
  void Init(const boost::property_tree::ptree &pt);

  int Search();

  int Evaluate(const std::vector<int> &state, int node,
               std::vector<int> &values);

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable, std::vector<int> &values,
               std::unordered_set<int> &preferred);

  void DumpPreferringMetrics() const;

  bool use_preferred_;
  bool same_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int n_branching_;
  int n_preferreds_;
  std::vector<int> plan_;
  std::vector<bool> is_preferred_action_;
  std::shared_ptr<const SASPlus> problem_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
  std::shared_ptr<Evaluator> preferring_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraph> graph_;
  std::unique_ptr<OpenList<std::vector<int>, std::pair<int, int> > > open_list_;
};

}  // namespace pplanner

#endif  // LAZY_GBFS_H_
