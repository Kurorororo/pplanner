#ifndef MRW13_MULTI_LMCOUNT_H_
#define MRW13_MULTI_LMCOUNT_H_

#include <random>
#include <vector>

#include "domain/domain.h"
#include "domain/state.h"
#include "heuristic/landmark_count.h"
#include "random_walk/local_restart.h"
#include "trie/trie.h"

extern int generated;
extern int expanded;
extern int evaluated;
extern int deadend;
extern int evaluations;

namespace rwls {

class Mrw13MultiLmcount {
 public:
  explicit Mrw13MultiLmcount(const Domain &domain) : eps_(0.1), tg_(1000) {
    ls_.push_back(10);
    ls_.push_back(100);
    ls_.push_back(1000);

    for (size_t i=0; i<ls_.size(); ++i)
      value_rls_.push_back(0);

    for (size_t  i=0; i<ls_.size(); ++i)
      cost_rls_.push_back(0);

    std::random_device seed_gen;
    engine_ = std::default_random_engine(seed_gen());
    domain_ = domain;
    table_ = ConstructTable(domain);
    lmcount_.Initialize(domain);
  }

  ~Mrw13MultiLmcount() {
    FinalizeTable(&table_);
  }

  std::vector<int> operator()(int n_walks=5120);

 private:
  int Walk(int h_min, int length, State &state,
           std::array<std::vector<bool>, 2> &accepted,
           std::vector<bool> &best_accepted, std::vector<int> &sequence);

  double eps_;
  int tg_;
  std::vector<int> ls_;
  std::vector<int> value_rls_;
  std::vector<int> cost_rls_;
  Domain domain_;
  TrieTable table_;
  LandmarkCount lmcount_;
  std::default_random_engine engine_;
};

} // namespace rwls

#endif // MRW13_MULTI_LMCOUNT_H_
