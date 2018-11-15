#ifndef PARALLEL_GBFS_SIMULATOR_H_
#define PARALLEL_GBFS_SIMULATOR_H_

#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "open_list.h"
#include "sas_plus.h"
#include "search.h"
#include "search_graph.h"
#include "successor_generator.h"

namespace pplanner {

class PGSimulator : public Search {
 public:
  PGSimulator(std::shared_ptr<const SASPlus> problem,
              const boost::property_tree::ptree &pt)
    : use_local_open_(false),
      n_processes_(1),
      max_expansion_(-1),
      best_h_(0),
      generated_(0),
      expanded_(0),
      evaluated_(0),
      dead_ends_(0),
      problem_(problem),
      generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
      graph_(nullptr) { Init(pt); }

  virtual ~PGSimulator() {}

  std::vector<int> Plan() override {
    int goal = Search();

    return ExtractPath(graph_, goal);
  }

  void DumpStatistics() const override;

  virtual int Search();

 private:
  void Init(const boost::property_tree::ptree &pt);

  bool NoNode() const;

  std::vector<int> InitialEvaluate();

  int Expand(int node, std::vector<int> &state);

  int Evaluate(const std::vector<int> &state, int node,
               std::vector<int> &values);

  bool use_local_open_;
  int n_processes_;
  int max_expansion_;
  int best_h_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  std::shared_ptr<const SASPlus> problem_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraph> graph_;
  std::vector<std::unique_ptr<OpenList> > open_lists_;
  std::vector<std::unique_ptr<OpenList> > local_open_lists_;
  std::shared_ptr<DistributionHash> z_hash_;
};

} // namespace pplanner

#endif // PARALLEL_GBFS_SIMULATOR_H_
