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
                std::shared_ptr<SearchGraph> search_graph,
                bool unit_cost=true, bool simplify=false,
                bool use_rpg_table=false, bool more_helpful=false)
    : search_graph_(search_graph),
      lmcount_(nullptr) {
    lmcount_ = std::unique_ptr<LandmarkCountBase>(
        new LandmarkCountBase(problem, unit_cost, simplify, use_rpg_table,
                              more_helpful));
    search_graph->InitLandmarks(lmcount_->landmark_graph());
  }

  int Evaluate(const std::vector<int> &state, int node) override {
    uint8_t *accepted = search_graph_->Landmark(node);
    uint8_t *parent_accepted = search_graph_->ParentLandmark(node);

    return lmcount_->Evaluate(state, parent_accepted, accepted);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent_node)
    override {
    uint8_t *accepted = search_graph_->Landmark(node);
    uint8_t *parent_accepted = search_graph_->Landmark(parent_node);

    return lmcount_->Evaluate(state, parent_accepted, accepted);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    uint8_t *accepted = search_graph_->Landmark(node);
    uint8_t *parent_accepted = search_graph_->ParentLandmark(node);

    return lmcount_->Evaluate(
        state, applicable, parent_accepted, accepted, preferred);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent_node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    uint8_t *accepted = search_graph_->Landmark(node);
    uint8_t *parent_accepted = search_graph_->Landmark(parent_node);

    return lmcount_->Evaluate(
        state, applicable, parent_accepted, accepted, preferred);
  }

  std::shared_ptr<const LandmarkGraph> landmark_graph() const {
    return lmcount_->landmark_graph();
  }

 private:
  std::shared_ptr<SearchGraph> search_graph_;
  std::unique_ptr<LandmarkCountBase> lmcount_;
};

class RWLandmarkCount : public RandomWalkEvaluator {
 public:
  RWLandmarkCount() : accepted_index_(0), lmcount_(nullptr) {
    lmcount_ = std::unique_ptr<LandmarkCountBase>(new LandmarkCountBase());
  }

  RWLandmarkCount(std::shared_ptr<const SASPlus> problem, bool unit_cost=true,
                  bool simplify=false, bool use_rpg_table=false,
                  bool more_helpful=false) : accepted_index_(0),
                                             lmcount_(nullptr) {
    lmcount_ = std::unique_ptr<LandmarkCountBase>(
        new LandmarkCountBase(problem, unit_cost, simplify, use_rpg_table,
                              more_helpful));

    auto graph = lmcount_->landmark_graph();
    int id_max = graph->landmark_id_max();
    best_accepted_.resize(id_max, false);
    accepted_[0].resize(id_max, false);
    accepted_[1].resize(id_max, false);
  }

  int Evaluate(const std::vector<int> &state) override {
    uint8_t *parent_accepted = nullptr;
    uint8_t *accepted = nullptr;
    PrepareEvaluation(&parent_accepted, &accepted);

    int h = lmcount_->Evaluate(state, parent_accepted, accepted);

    if (parent_accepted == nullptr)
      initial_accepted_ = accepted_[accepted_index_];

    return h;
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    uint8_t *parent_accepted = nullptr;
    uint8_t *accepted = nullptr;
    PrepareEvaluation(&parent_accepted, &accepted);

    int h = lmcount_->Evaluate(
        state, applicable, parent_accepted, accepted, preferred);

    if (parent_accepted == nullptr)
      initial_accepted_ = accepted_[accepted_index_];

    return h;
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
  void PrepareEvaluation(uint8_t **parent_accepted, uint8_t **accepted) {
    std::fill(accepted_[accepted_index_].begin(),
              accepted_[accepted_index_].end(), false);
    *accepted = accepted_[accepted_index_].data();

    if (!initial_accepted_.empty())
      *parent_accepted = accepted_[1 - accepted_index_].data();

    accepted_index_ = 1 - accepted_index_;
  }

  int accepted_index_;
  std::vector<uint8_t> initial_accepted_;
  std::vector<uint8_t> best_accepted_;
  std::array<std::vector<uint8_t>, 2> accepted_;
  std::unique_ptr<LandmarkCountBase> lmcount_;
};

} // namespace pplanner

#endif // LANDMARK_COUNT_H_
