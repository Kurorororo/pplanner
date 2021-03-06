#ifndef LANDMARK_COUNT_BASE_H_
#define LANDMARK_COUNT_BASE_H_

#include <cstdint>

#include <iostream>
#include <unordered_set>
#include <vector>

#include "heuristics/rpg.h"
#include "heuristics/rpg_factory.h"
#include "landmark/generating_orderings.h"
#include "landmark/landmark_detection.h"
#include "landmark/landmark_graph.h"
#include "sas_plus.h"

namespace pplanner {

class LandmarkCountBase {
 public:
  LandmarkCountBase()
      : problem_(nullptr),
        r_problem_(nullptr),
        rpg_(nullptr),
        graph_(nullptr) {}

  LandmarkCountBase(std::shared_ptr<const SASPlus> problem, bool unit_cost,
                    bool simplify, bool use_rpg_table, bool more_helpful)
      : problem_(problem),
        r_problem_(
            std::make_shared<RelaxedSASPlus>(problem, simplify, unit_cost)),
        rpg_(nullptr),
        graph_(std::make_shared<LandmarkGraph>(problem)) {
    auto cpp = "cpp";
    rpg_ = RPGFactory(problem, r_problem_, use_rpg_table, cpp, more_helpful);
    IdentifyLandmarks(problem, r_problem_, graph_, use_rpg_table);
    AddOrderings(problem, r_problem_, graph_);
    HandleCycles(graph_);
    status_.resize(graph_->landmark_id_max(), NOT_REACHED);
  }

  int Evaluate(const std::vector<int> &state, const uint8_t *parent_accepted,
               uint8_t *accepted);

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               const uint8_t *parent_accepted, uint8_t *accepted,
               std::unordered_set<int> &preferred,
               bool next_step_onluy = false);

  std::shared_ptr<LandmarkGraph> landmark_graph() const { return graph_; }

  int FFEvaluate(const std::vector<int> &state,
                 std::unordered_set<int> &preferred) {
    StateToFactVector(problem_, state, facts_);

    return rpg_->PlanCost(facts_, preferred);
  }

 private:
  bool IsLeaf(int lm_id, const uint8_t *accepted) const;

  int ReachedSize(const std::vector<int> &state, const uint8_t *parent_accepted,
                  uint8_t *accepted);

  int NeededSize(const std::vector<int> &state, const uint8_t *accepted);

  bool IsInteresting(int lm_id, const std::vector<int> &state,
                     const uint8_t *accepted) const;

  void NextStepOperators(const std::vector<int> &state,
                         const std::vector<int> &applicable,
                         const uint8_t *accepted,
                         std::unordered_set<int> &preferred);

  enum LandmarkState {
    REACHED,
    NEEDED,
    NOT_REACHED,
  };

  int reached_size_;
  std::vector<int> facts_;
  std::vector<LandmarkState> status_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::shared_ptr<RPG> rpg_;
  std::shared_ptr<LandmarkGraph> graph_;
};

}  // namespace pplanner

#endif  // LANDMARK_COUNT_BASE_H_
