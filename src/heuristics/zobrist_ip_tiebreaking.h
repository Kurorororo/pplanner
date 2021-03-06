#ifndef ZOBRIST_IP_TIEBREAKING_H_
#define ZOBRIST_IP_TIEBREAKING_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "hash/zobrist_hash.h"

namespace pplanner {

class ZobristIPTiebreaking : public Evaluator {
 public:
  ZobristIPTiebreaking() {}

  ZobristIPTiebreaking(std::shared_ptr<const SASPlus> problem)
      : hash_(std::unique_ptr<ZobristHash>(
            new ZobristHash(problem, 2968297125))) {}

  ~ZobristIPTiebreaking() {}

  int Evaluate(const std::vector<int> &state, int node) override {
    uint32_t r = hash_->operator()(state);

    return static_cast<int>(r >> 1);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node);
  }

  // for MPI
  int Evaluate(const std::vector<int> &state, int node, int parent) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node, applicable, preferred);
  }

  // for multithread
  int Evaluate(const std::vector<int> &state,
               std::shared_ptr<SearchNode> node) override {
    return Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state, std::shared_ptr<SearchNode> node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, -1, applicable, preferred);
  }
  // for random walk
  int Evaluate(const std::vector<int> &state) override {
    return Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, -1, applicable, preferred);
  }

  void UpdateBest() override {}

  void LocalRestart() override {}

  void GlobalRestart() override {}

  void CopyBestToSearchGraph(int node,
                             std::shared_ptr<SearchGraph> graph) override {}

 private:
  std::unique_ptr<ZobristHash> hash_;
};

}  // namespace pplanner

#endif  // ZOBRIST_IP_TIEBREAKING_H_
