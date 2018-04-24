#ifndef MRW13_MULTI_H_
#define MRW13_MULTI_H_

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
class Mrw13Multi {
 public:
  Mrw13Multi() : eps_(0.1) {
    ls_.push_back(10);
    ls_.push_back(100);
    ls_.push_back(1000);

    for (size_t i=0; i<ls_.size(); ++i)
      value_rls_.push_back(0);

    for (size_t  i=0; i<ls_.size(); ++i)
      cost_rls_.push_back(0);
  };

  explicit Mrw13Multi(const Domain &domain, bool uniform=false)
        : eps_(0.1), tg_(1000), e1_(exp(0.1)), ew_(exp(0.1)) {
    uniform_ = uniform;

    ls_.push_back(10);
    ls_.push_back(100);
    ls_.push_back(1000);

    q1_.resize(domain.action_size, 1.0);
    qw_.resize(domain.action_size, 1.0);

    for (size_t i=0; i<ls_.size(); ++i)
      value_rls_.push_back(0);

    for (size_t i=0; i<ls_.size(); ++i)
      cost_rls_.push_back(0);

    std::random_device seed_gen;
    engine_ = std::default_random_engine(seed_gen());
    domain_ = domain;
    table_ = ConstructTable(domain);
    heuristic_.Initialize(domain);
  }

  ~Mrw13Multi() {
    FinalizeTable(&table_);
  }

  std::vector<int> operator()(int n_walks);

 private:
  int MHA(const std::vector<int> &applicable,
          std::unordered_set<int> &preferred);

  void UpdateQ(const std::vector<int> &applicable,
               const std::unordered_set<int> &preferred);

  int Walk(int h_min, int length, State &state, std::vector<int> &sequence,
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
  std::vector<int> ls_;
  std::vector<int> value_rls_;
  std::vector<int> cost_rls_;
  std::vector<double> q1_;
  std::vector<double> qw_;
  std::default_random_engine engine_;
};

} // namespace rwls

#include "details/mrw13_multi.h"

#endif // MRW13_MULTI_H_
