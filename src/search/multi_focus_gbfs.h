#ifndef MULTI_FOCUS_MRW_GBFS_H_
#define MULTI_FOCUS_MRW_GBFS_H_

#include <cmath>

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

class MultiFocusMrwGBFS : public Search {
 public:
  MultiFocusMrwGBFS(std::shared_ptr<const SASPlus> problem,
                    const boost::property_tree::ptree &pt)
      : uniform_(false),
        use_preferred_(false),
        generated_(0),
        expanded_(0),
        evaluated_(0),
        dead_ends_(0),
        rw_evaluated_(0),
        tg_(1000),
        n_restarts_(0),
        best_h_(-1),
        rw_initial_h_(-1),
        rw_best_h_(-1),
        eps_(0.1),
        e1_(exp(0.1)),
        ew_(exp(0.1)),
        v_w_(0.0),
        rls_({0.1, 0.01, 0.001}),
        ls_({10, 100, 1000}),
        value_rls_(3, 0),
        cost_rls_(3, 0),
        q1_(problem->n_actions(), 1.0),
        qw_(problem->n_actions(), 1.0),
        dist_(0.0, 1.0),
        problem_(problem),
        generator_(std::unique_ptr<SuccessorGenerator>(
            new SuccessorGenerator(problem))),
        rw_evaluator_(nullptr),
        graph_(nullptr),
        preferring_(nullptr) {
    Init(pt);
  }

  ~MultiFocusMrwGBFS() {}

  std::vector<int> Plan() override;

  void DumpStatistics() const override;

 private:
  void Init(const boost::property_tree::ptree &pt);

  void RWInitialEvaluate();

  int RWEvaluate(const std::vector<int> &state,
                 const std::vector<int> &applicable,
                 std::unordered_set<int> &preferred);

  void UpdateQ(const std::vector<int> &applicable,
               const std::unordered_set<int> &preferred);

  int MHA(const std::vector<int> &applicable,
          std::unordered_set<int> &preferred);

  void AddNewFocus(int h, const std::vector<int> &state,
                   const std::vector<int> &sequence);

  void UpdateBest(int h, const std::vector<int> &state,
                  const std::vector<int> &applicable,
                  const std::unordered_set<int> &preferred,
                  const std::vector<int> &sequence);

  void GlobalRestart(int li);

  bool Walk(int *w, int *li);

  int Evaluate(const std::vector<int> &state, int node,
               std::vector<int> &values);

  void InitialEvaluate();

  std::shared_ptr<OpenList<> > GreedyOpen();

  int Expand();

  std::vector<int> ExtractPlan(int node);

  bool uniform_;
  bool use_preferred_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int rw_evaluated_;
  int tg_;
  int n_restarts_;
  int best_h_;
  int rw_initial_h_;
  int rw_best_h_;
  double eps_;
  double qw_max_;
  double e1_;
  double ew_;
  double v_w_;
  std::vector<int> rw_initial_applicable_;
  std::unordered_set<int> rw_initial_preferred_;
  std::vector<int> rw_best_state_;
  std::vector<int> rw_best_applicable_;
  std::unordered_set<int> rw_best_preferred_;
  std::vector<int> rw_plan_;
  std::vector<double> rls_;
  std::vector<int> ls_;
  std::vector<int> value_rls_;
  std::vector<int> cost_rls_;
  std::vector<double> q1_;
  std::vector<double> qw_;
  std::mt19937 engine_;
  std::uniform_real_distribution<> dist_;
  std::shared_ptr<const SASPlus> problem_;
  std::unique_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<Evaluator> rw_evaluator_;
  std::unordered_map<int, std::vector<int> > rw_plan_to_node_;
  std::shared_ptr<SearchGraph> graph_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
  std::shared_ptr<Evaluator> preferring_;
  boost::property_tree::ptree open_list_option_;
  std::vector<std::shared_ptr<OpenList<> > > open_lists_;
};

}  // namespace pplanner

#endif  // MULTI_FOCUS_MRW_GBFS_H_
