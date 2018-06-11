#ifndef LANDMARK_COUNT_BASE_H_
#define LANDMARK_COUNT_BASE_H_

#include <cstdint>

#include <unordered_set>
#include <vector>

#include "sas_plus.h"
#include "landmark/generating_orderings.h"
#include "landmark/landmark_detection.h"
#include "landmark/landmark_graph.h"
#include "heuristics/rpg.h"

namespace pplanner {

class LandmarkCountBase {
 public:
  LandmarkCountBase() : problem_(nullptr),
                        r_problem_(nullptr),
                        rpg_(nullptr),
                        landmark_graph_(nullptr) {}

  LandmarkCountBase(std::shared_ptr<const SASPlus> problem, bool simplify)
    : problem_(problem),
      r_problem_(std::make_shared<RelaxedSASPlus>(*problem, simplify)),
      rpg_(nullptr),
      landmark_graph_(std::make_shared<LandmarkGraph>()) {
    rpg_ = std::unique_ptr<RPG>(new RPG(r_problem_));
    IdentifyLandmarks(problem, r_problem_, landmark_graph_);
    AddOrderings(problem, r_problem_, landmark_graph_);
    HandleCycles(landmark_graph_);
    status_.resize(landmark_graph_->landmark_id_max(), NOT_REACHED);
  }

  int Evaluate(const std::vector<int> &state, const uint8_t *parent_accepted,
               uint8_t *accepted);

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               const uint8_t *parent_accepted,
               uint8_t *accepted,
               std::unordered_set<int> &preferred) {
    return Evaluate(state, parent_accepted, accepted);
  }

  std::shared_ptr<LandmarkGraph> landmark_graph() const {
    return landmark_graph_;
  }

 private:
  int ReachedSize(const std::vector<int> &state, const uint8_t *parent_accepted,
                  uint8_t *accepted);

  int NeededSize(const std::vector<int> &state, const uint8_t *accepted);

  enum LandmarkState { REACHED, NEEDED, NOT_REACHED, };

  std::vector<LandmarkState> status_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<RelaxedSASPlus> r_problem_;
  std::shared_ptr<RPG> rpg_;
  std::shared_ptr<LandmarkGraph> landmark_graph_;
};

} // namespace pplanner

#endif // LANDMARK_COUNT_BASE_H_
