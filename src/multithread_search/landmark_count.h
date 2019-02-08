#ifndef M_LANDMARK_COUNT_H_
#define M_LANDMARK_COUNT_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "landmark/landmark_count_base.h"
#include "multithread_search/heuristic.h"
#include "sas_plus.h"

namespace pplanner {

class MLandmarkCount : public Heuristic {
 public:
  MLandmarkCount() : lmcount_(nullptr) {
    lmcount_ = std::make_unique<LandmarkCountBase>();
  }

  MLandmarkCount(std::shared_ptr<const SASPlus> problem,
                 bool unit_cost=true, bool use_rpg_table=false,
                 bool more_helpful=false)
    : lmcount_(nullptr) {
    lmcount_ = std::make_unique<LandmarkCountBase>(
        problem, unit_cost, false, use_rpg_table, more_helpful);
    std::size_t id_max = lmcount_->landmark_graph()->landmark_id_max();
    n_landmark_bytes_ = (id_max + 7) / 8;
  }

  int Evaluate(const std::vector<int> &state,
               std::shared_ptr<SearchNode> node) override {
    node->landmark.resize(n_landmark_bytes_);
    uint8_t *accepted = node->landmark.data();
    uint8_t *parent_accepted = nullptr;

    if (node->parent != nullptr)
      parent_accepted = node->parent->landmark.data();

    return lmcount_->Evaluate(state, parent_accepted, accepted);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred,
               std::shared_ptr<SearchNode> node) override {
    node->landmark.resize(n_landmark_bytes_);
    uint8_t *accepted = node->landmark.data();
    uint8_t *parent_accepted = nullptr;

    if (node->parent != nullptr)
      parent_accepted = node->parent->landmark.data();

    return lmcount_->Evaluate(
        state, applicable, parent_accepted, accepted, preferred);
  }

 private:
  std::size_t n_landmark_bytes_;
  std::unique_ptr<LandmarkCountBase> lmcount_;
};

} // namespace pplanner

#endif // M_LANDMARK_COUNT_H_
