#ifndef LANDMARK_COUNT_H_
#define LANDMARK_COUNT_H_

#include <array>
#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "random_walk_evaluator.h"
#include "sas_plus.h"
#include "landmark/landmark_count_base.h"
#include "search_graph/search_graph_with_landmarks.h"

namespace pplanner {

class LandmarkCount : public Evaluator {
 public:
  LandmarkCount() : search_graph_(nullptr), lmcount_(nullptr) {
    lmcount_ = std::unique_ptr<LandmarkCountBase>(new LandmarkCountBase());
  }

  LandmarkCount(std::shared_ptr<const SASPlus> problem,
                std::shared_ptr<SearchGraphWithLandmarks> search_graph,
                bool simplify=false) : search_graph_(search_graph),
                                       lmcount_(nullptr){
    lmcount_ = std::unique_ptr<LandmarkCountBase>(
        new LandmarkCountBase(problem, simplify));
    search_graph->InitLandmarks(lmcount_->landmark_graph());
  }

  int Evaluate(const std::vector<int> &state, int node) override {
    uint8_t *accepted = search_graph_->Landmark(node);
    uint8_t *parent_accepted = nullptr;

    if (node > 0) {
      int parent = search_graph_->Parent(node);
      parent_accepted = search_graph_->Landmark(parent);
    }

    return lmcount_->Evaluate(state, parent_accepted, accepted);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node);
  }

 private:
  std::shared_ptr<SearchGraphWithLandmarks> search_graph_;
  std::unique_ptr<LandmarkCountBase> lmcount_;
};

class RWLandmarkCount : public RandomWalkEvaluator {
 public:
  RWLandmarkCount() : accepted_index_(0), lmcount_(nullptr) {
    lmcount_ = std::unique_ptr<LandmarkCountBase>(new LandmarkCountBase());
  }

  RWLandmarkCount(std::shared_ptr<const SASPlus> problem, bool simplify=false)
    : accepted_index_(0), lmcount_(nullptr) {
    lmcount_ = std::unique_ptr<LandmarkCountBase>(
        new LandmarkCountBase(problem, simplify));

    auto graph = lmcount_->landmark_graph();
    size_t id_max = graph->landmark_id_max();
    best_accepted_.resize(id_max, false);
    accepted_[0].resize(id_max, false);
    accepted_[1].resize(id_max, false);
  }

  int Evaluate(const std::vector<int> &state) override {
    uint8_t *accepted = accepted_[accepted_index_].data();
    uint8_t *parent_accepted = nullptr;

    if (!initial_accepted_.empty()) {
      parent_accepted = accepted_[1 - accepted_index_].data();
      accepted_index_ = 1 - accepted_index_;

      return lmcount_->Evaluate(state, parent_accepted, accepted);
    }

    int h = lmcount_->Evaluate(state, parent_accepted, accepted);
    initial_accepted_ = accepted_[accepted_index_];
    accepted_index_ = 1 - accepted_index_;

    return h;
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state);
  }

  void UpdateBest() override {
    best_accepted_ = accepted_[1 - accepted_index_];
  }

  void LocalRestart() override {
    accepted_index_ = 1;
    accepted_[0] = best_accepted_;
  }

  void GlobalRestart() override {
    initial_accepted_.clear();
    accepted_index_ = 0;
  }

 private:
  int accepted_index_;
  std::vector<uint8_t> initial_accepted_;
  std::vector<uint8_t> best_accepted_;
  std::array<std::vector<uint8_t>, 2> accepted_;
  std::unique_ptr<LandmarkCountBase> lmcount_;
};

} // namespace pplanner

#endif // LANDMARK_COUNT_H_
