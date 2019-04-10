#ifndef LANDMARK_COUNT_H_
#define LANDMARK_COUNT_H_

#include <array>
#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "landmark/landmark_count_base.h"
#include "sas_plus.h"
#include "search_graph.h"
#include "search_node.h"

namespace pplanner {

class LandmarkCount : public Evaluator {
 public:
  LandmarkCount()
      : lmcount_(std::make_unique<LandmarkCountBase>()),
        search_graph_(nullptr) {}

  LandmarkCount(std::shared_ptr<const SASPlus> problem,
                std::shared_ptr<SearchGraph> search_graph,
                bool unit_cost = true, bool simplify = false,
                bool use_rpg_table = false, bool more_helpful = false)
      : lmcount_(std::make_unique<LandmarkCountBase>(
            problem, unit_cost, simplify, use_rpg_table, more_helpful)),
        search_graph_(search_graph) {
    auto graph = lmcount_->landmark_graph();

    std::size_t id_max = graph->landmark_id_max();
    n_landmark_bytes_ = (id_max + 7) / 8;

    best_accepted_.resize(id_max, false);
    accepted_[0].resize(id_max, false);
    accepted_[1].resize(id_max, false);

    if (search_graph != nullptr) search_graph->InitLandmarks(graph);
  }

  int Evaluate(const std::vector<int> &state, int node) override {
    uint8_t *accepted = search_graph_->Landmark(node);
    uint8_t *parent_accepted = search_graph_->ParentLandmark(node);

    return lmcount_->Evaluate(state, parent_accepted, accepted);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    uint8_t *accepted = search_graph_->Landmark(node);
    uint8_t *parent_accepted = search_graph_->ParentLandmark(node);

    return lmcount_->Evaluate(state, applicable, parent_accepted, accepted,
                              preferred);
  }

  // for MPI
  int Evaluate(const std::vector<int> &state, int node,
               int parent_node) override {
    uint8_t *accepted = search_graph_->Landmark(node);
    uint8_t *parent_accepted = search_graph_->Landmark(parent_node);

    return lmcount_->Evaluate(state, parent_accepted, accepted);
  }

  int Evaluate(const std::vector<int> &state, int node, int parent_node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    uint8_t *accepted = search_graph_->Landmark(node);
    uint8_t *parent_accepted = search_graph_->Landmark(parent_node);

    return lmcount_->Evaluate(state, applicable, parent_accepted, accepted,
                              preferred);
  }

  // for multithread
  int Evaluate(const std::vector<int> &state, SearchNode *node) override {
    node->landmark.resize(n_landmark_bytes_);
    uint8_t *accepted = node->landmark.data();
    uint8_t *parent_accepted = nullptr;

    if (node->parent != nullptr)
      parent_accepted = node->parent->landmark.data();

    return lmcount_->Evaluate(state, parent_accepted, accepted);
  }

  int Evaluate(const std::vector<int> &state, SearchNode *node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    node->landmark.resize(n_landmark_bytes_);
    uint8_t *accepted = node->landmark.data();
    uint8_t *parent_accepted = nullptr;

    if (node->parent != nullptr)
      parent_accepted = node->parent->landmark.data();

    return lmcount_->Evaluate(state, applicable, parent_accepted, accepted,
                              preferred);
  }

  // for random walk
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

    int h = lmcount_->Evaluate(state, applicable, parent_accepted, accepted,
                               preferred);

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

  void CopyBestToSearchGraph(int node,
                             std::shared_ptr<SearchGraph> graph) override {
    auto landmark = graph->Landmark(node);
    memcpy(landmark, best_accepted_.data(),
           best_accepted_.size() * sizeof(uint8_t));
  }

  std::shared_ptr<const LandmarkGraph> landmark_graph() const {
    return lmcount_->landmark_graph();
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
  std::size_t n_landmark_bytes_;
  std::vector<uint8_t> initial_accepted_;
  std::vector<uint8_t> best_accepted_;
  std::array<std::vector<uint8_t>, 2> accepted_;
  std::unique_ptr<LandmarkCountBase> lmcount_;
  std::shared_ptr<SearchGraph> search_graph_;
};

}  // namespace pplanner

#endif  // LANDMARK_COUNT_H_
