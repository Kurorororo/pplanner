#ifndef LANDMARK_COUNT_H_
#define LANDMARK_COUNT_H_

#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "sas_plus.h"
#include "landmark/generating_orderings.h"
#include "landmark/landmark_detection.h"
#include "landmark/landmark_graph.h"
#include "heuristics/rpg.h"
#include "search_graph/search_graph_with_landmarks.h"

namespace pplanner {

class LandmarkCount : public Evaluator {
 public:
  LandmarkCount() : problem_(nullptr),
                    r_problem_(nullptr),
                    search_graph_(nullptr),
                    rpg_(nullptr),
                    landmark_graph_(nullptr) {}

  LandmarkCount(std::shared_ptr<const SASPlus> problem,
                std::shared_ptr<SearchGraphWithLandmarks> search_graph,
                bool simplify=false)
    : problem_(problem),
      r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
      search_graph_(search_graph),
      rpg_(nullptr),
      landmark_graph_(std::make_shared<LandmarkGraph>()) {
    rpg_ = std::unique_ptr<RPG>(new RPG(r_problem_));
    IdentifyLandmarks(problem, r_problem_, landmark_graph_);
    AddOrderings(problem, r_problem_, landmark_graph_);
    HandleCycles(landmark_graph_);
    status_.resize(landmark_graph_->landmark_id_max(), NOT_REACHED);
    search_graph->InitLandmarks(landmark_graph_);
  }

  int Evaluate(const std::vector<int> &state, int node) override;

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node);
  }


 private:
  int ReachedSize(const std::vector<int> &state, const uint8_t *parent_accepted,
                  uint8_t *accepted);

  int NeededSize(const std::vector<int> &state, const uint8_t *accepted);

  enum LandmarkState { REACHED, NEEDED, NOT_REACHED, };

  std::vector<LandmarkState> status_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::shared_ptr<SearchGraphWithLandmarks> search_graph_;
  std::shared_ptr<RPG> rpg_;
  std::shared_ptr<LandmarkGraph> landmark_graph_;
};

} // namespace pplanner

#endif // LANDMARK_COUNT_H_
