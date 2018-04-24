#ifndef MRW13_H_
#define MRW13_H_

#include <cmath>

#include <random>
#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "domain/state.h"
#include "random_walk/local_restart.h"
#include "trie/trie.h"

extern int generated;
extern int expanded;
extern int evaluated;
extern int deadend;
extern int evaluations;

namespace rwls {

template<class H>
class Mrw13 {
 public:
  Mrw13() : eps_(0.1) {
    rls_.push_back(0.1);
    rls_.push_back(0.01);
    rls_.push_back(0.001);

    ls_.push_back(10);
    ls_.push_back(100);
    ls_.push_back(1000);

    for (size_t i=0; i<rls_.size(); ++i)
      value_rls_.push_back(0);

    for (size_t  i=0; i<rls_.size(); ++i)
      cost_rls_.push_back(0);
  };

  explicit Mrw13(const Domain &domain, bool uniform=false)
        : eps_(0.1), tg_(1000), e1_(exp(0.1)), ew_(exp(0.1)) {
    uniform_ = uniform;
    rls_.push_back(0.1);
    rls_.push_back(0.01);
    rls_.push_back(0.001);

    ls_.push_back(10);
    ls_.push_back(100);
    ls_.push_back(1000);

    q1_.resize(domain.action_size, 1.0);
    qw_.resize(domain.action_size, 1.0);

    for (size_t i=0; i<rls_.size(); ++i)
      value_rls_.push_back(0);

    for (size_t i=0; i<rls_.size(); ++i)
      cost_rls_.push_back(0);

    std::random_device seed_gen;
    engine_ = std::default_random_engine(seed_gen());
    domain_ = domain;
    table_ = ConstructTable(domain);
    heuristic_.Initialize(domain);
  }

  ~Mrw13() {
    FinalizeTable(&table_);
  }

  std::vector<int> operator()(bool fix=false);

 private:
  int MHA(const std::vector<int> &applicable,
          std::unordered_set<int> &preferred);

  void UpdateQ(const std::vector<int> &applicable,
               const std::unordered_set<int> &preferred);

  int Walk(int h_min, int length, State &state, std::vector<int> &sequence,
           std::unordered_set<int> &preferred, std::vector<int> &applicable);

  int Walk(int h_min, double rl, State &state, std::vector<int> &sequence,
           std::unordered_set<int> &preferred, std::vector<int> &applicable);

  int tg_;
  bool uniform_;
  double eps_;
  double qw_max_;
  double e1_;
  double ew_;
  Domain domain_;
  TrieTable table_;
  H heuristic_;
  std::vector<double> rls_;
  std::vector<int> ls_;
  std::vector<int> value_rls_;
  std::vector<int> cost_rls_;
  std::vector<double> q1_;
  std::vector<double> qw_;
  std::default_random_engine engine_;
};

} // namespace rwls

#include "details/mrw13.h"

#endif // MRW13_H_
