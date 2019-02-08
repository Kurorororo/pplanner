#ifndef ORBIT_GBFS_H_
#define ORBIT_GBFS_H_

#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "sas_plus.h"
#include "sas_plus/strong_stubborn_sets.h"
#include "search.h"
#include "search_graph.h"
#include "successor_generator.h"
#include "symmetry/symmetry.h"
#include "open_list.h"

namespace pplanner {

class OrbitGBFS : public Search {
 public:
  OrbitGBFS(std::shared_ptr<const SASPlus> problem,
       const boost::property_tree::ptree &pt)
    : use_preferred_(false),
      exhaust_(false),
      limit_expansion_(false),
      use_sss_(false),
      max_expansion_(0),
      generated_(0),
      expanded_(0),
      evaluated_(0),
      dead_ends_(0),
      n_preferred_evaluated_(0),
      n_branching_(0),
      n_preferreds_(0),
      problem_(problem),
      preferring_(nullptr),
      generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
      graph_(nullptr),
      open_list_(nullptr),
      manager_(std::make_shared<SymmetryManager>(problem)),
      sss_aproximater_(nullptr) { Init(pt); }

  virtual ~OrbitGBFS() {}

  std::vector<int> Plan() override {
    int goal = Search();
    auto trace = Trace(goal);

    return TraceForward(trace);
  }

  void DumpStatistics() const override;

  virtual int Search();

  bool NoNode() const { return open_list_->IsEmpty(); }

  int NodeToExpand() { return open_list_->Pop(); }

  std::vector<int> InitialExpand();

  int Expand(int node, std::vector<int> &state, std::vector<int> &child,
             std::vector<int> &applicable, std::unordered_set<int> &preferred);

  std::vector<std::pair<int, int> > Trace(int goal) const;

  std::vector<int> TraceForward(const std::vector<std::pair<int, int> > &trace)
    const;

 private:
  void Init(const boost::property_tree::ptree &pt);

  bool use_preferred_;
  bool exhaust_;
  bool limit_expansion_;
  bool use_sss_;
  int max_expansion_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int n_preferred_evaluated_;
  int n_branching_;
  int n_preferreds_;
  int best_h_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<Evaluator> preferring_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraph> graph_;
  std::unique_ptr<OpenList<int> > open_list_;
  std::shared_ptr<SymmetryManager> manager_;
  std::unique_ptr<SSSApproximater> sss_aproximater_;
};

} // namespace pplanner

#endif // ORIBT_GBFS_H_
