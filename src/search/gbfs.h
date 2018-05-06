#ifndef GBFS_H_
#define GBFS_H_

#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "sas_plus.h"
#include "search.h"
#include "search_graph.h"
#include "successor_generator.h"
#include "open_list.h"

namespace pplanner {

class GBFS : public Search {
 public:
  GBFS(std::shared_ptr<const SASPlus> problem,
       const boost::property_tree::ptree &pt)
    : use_preferred_(false),
      generated_(0),
      expanded_(0),
      evaluated_(0),
      dead_ends_(0),
      n_preferreds_(0),
      problem_(problem),
      preferring_(nullptr),
      generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
      graph_(std::unique_ptr<SearchGraph>(new SearchGraph(*problem_))),
      open_list_(nullptr) { Init(pt); }

  ~GBFS() {}

  std::vector<int> Plan() override {
    int goal = Search();

    return ExtractPath(*graph_, goal);
  }

  void DumpStatistics() const override;

 private:
  void Init(const boost::property_tree::ptree &pt);

  int Search();

  bool use_preferred_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int n_preferreds_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<Evaluator> preferring_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::unique_ptr<SearchGraph> graph_;
  std::unique_ptr<OpenList> open_list_;
};

} // namespace pplanner

#endif // GBFS_H_
