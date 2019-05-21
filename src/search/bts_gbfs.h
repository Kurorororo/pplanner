#ifndef BTSGBFS_H_
#define BTSGBFS_H_

#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "open_list.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph.h"
#include "successor_generator.h"

namespace pplanner {

class BTSGBFS : public Search {
 public:
  BTSGBFS(std::shared_ptr<const SASPlus> problem,
          const boost::property_tree::ptree &pt)
      : generated_(0),
        expanded_(0),
        evaluated_(0),
        dead_ends_(0),
        hwm_(-1),
        problem_(problem),
        generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
        graph_(nullptr),
        open_list_(nullptr) {
    Init(pt);
  }

  virtual ~BTSGBFS() {}

  std::vector<int> Plan() override {
    int goal = Search();
    auto plan = ExtractPath(graph_, goal);

    return plan;
  }

  void DumpStatistics() const override;

  virtual int Search();

  bool NoNode() const { return open_list_->IsEmpty(); }

  std::vector<int> InitialExpand();

  int Expand(std::vector<int> &state, std::vector<int> &child,
             std::vector<int> &applicable, bool plan_found);

 private:
  void Init(const boost::property_tree::ptree &pt);

  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int best_h_;
  int hwm_;
  std::shared_ptr<const SASPlus> problem_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraph> graph_;
  std::unique_ptr<OpenList<> > open_list_;
};

}  // namespace pplanner

#endif  // BTSGBFS_H_
