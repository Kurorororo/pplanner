#ifndef MRW13_H_
#define MRW13_H_

#include <cmath>

#include <memory>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "random_walk_evaluator.h"
#include "sas_plus.h"
#include "search.h"
#include "successor_generator.h"

namespace pplanner {

class Mrw13 : public Search {
 public:
  Mrw13(std::shared_ptr<const SASPlus> problem,
        const boost::property_tree::ptree &pt,
        int max_expansion)
    : uniform_(false),
      fix_(false),
      same_(false),
      measure_(false),
      action_elimination_(false),
      solved_(false),
      generated_(0),
      expanded_(0),
      evaluated_(0),
      dead_ends_(0),
      n_branching_(0),
      n_preferreds_(0),
      max_expansion_(max_expansion),
      tg_(1000),
      n_restarts_(0),
      eps_(0.1),
      e1_(exp(0.1)),
      ew_(exp(0.1)),
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
      evaluator_(nullptr),
      preferring_(nullptr) { Init(pt); }

  ~Mrw13() {}

  std::vector<int> Plan() override;

  void DumpStatistics() const override;

 private:
  void Init(const boost::property_tree::ptree &pt);

  int Evaluate(const std::vector<int> &state, const std::vector<int> &applicable,
               std::unordered_set<int> &preferred);

  int MHA(const std::vector<int> &applicable,
          std::unordered_set<int> &preferred);

  void UpdateQ(const std::vector<int> &applicable,
               const std::unordered_set<int> &preferred);

  int Walk(int best_h, int length, std::vector<int> &state,
           std::vector<int> &sequence, std::vector<int> &applicable,
           std::unordered_set<int> &preferred);

  int Walk(int best_h, double rl, std::vector<int> &state,
           std::vector<int> &sequence,  std::vector<int> &applicable,
           std::unordered_set<int> &preferred);

  void ActionElimination();

  void DumpPreferringMetrics() const;

  bool uniform_;
  bool fix_;
  bool same_;
  bool measure_;
  bool action_elimination_;
  bool solved_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int n_branching_;
  int n_preferreds_;
  int max_expansion_;
  int tg_;
  int n_restarts_;
  double eps_;
  double qw_max_;
  double e1_;
  double ew_;
  std::vector<int> plan_;
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
  std::shared_ptr<RandomWalkEvaluator> evaluator_;
  std::shared_ptr<RandomWalkEvaluator> preferring_;
  std::vector<bool> is_preferred_operator_;
  std::vector<bool> is_preferred_successor_;
  std::vector<bool> tmp_is_preferred_successor_;
  std::vector<int> n_preferred_successors_;
  std::vector<int> tmp_n_preferred_successors_;
  std::vector<int> n_successors_;
  std::vector<int> tmp_n_successors_;
};

void Feedback(int arg_rl, int value, int cost, std::vector<int> &value_rls,
              std::vector<int> &cost_rls);

int RandomRl(std::size_t size, std::mt19937 &engine);

int ChoiceRl(double eps, const std::vector<int> &value_rls,
             const std::vector<int> &cost_rls, std::mt19937 &engine);

template<typename T>
T GetRl(int arg_rl, const std::vector<T> &rls) {
  return rls[arg_rl];
}

} // namespace pplanner

#endif // MRW13_H_
