#ifndef LANDMARK_COUNT_H_
#define LANDMARK_COUNT_H_

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "domain/domain.h"
#include "heuristic/heuristic.h"
#include "landmark/generating_orderings.h"
#include "landmark/landmark_detection.h"
#include "landmark/landmark_graph.h"

namespace rwls {

class LandmarkCount : public HeuristicInterface<LandmarkCount> {
 public:
  void Initialize(const Domain &domain) {
    state_.resize(domain.variables_size);

    GraphSchema schema;
    InitializeSchema(domain, &schema);
    IdentifyLandmarks(domain, schema, &graph);
    AddOrderings(domain, schema, &graph);
    HandleCycles(&graph);
    landmarks_size_ = static_cast<int>(graph.GetLandmarksSize());
    landmark_id_max = graph.GetLandmarks().size();
    accepted_bytes = (landmark_id_max + 7) / 8;
    status_.resize(landmark_id_max);
  }

  int operator()(const State &state, const Domain &domain,
                 const uint8_t *parent_accepted, uint8_t *accepted);

  int operator()(const State &state, const Domain &domain,
                 const std::vector<bool> &parent_accepted,
                 std::vector<bool> &accepted, bool initial=false);

  size_t landmark_id_max;
  size_t accepted_bytes;
  LandmarkGraph graph;

 private:
  int ReachedSize(const State &state, const uint8_t *parent_accepted,
                  uint8_t *accepted);

  int ReachedSize(const State &state, const std::vector<bool> &parent_accepted,
                  std::vector<bool> &accepted);

  int NeededSize(const State &state, const Domain &domain,
                 const uint8_t *accepted);

  int NeededSize(const State &state, const Domain &domain,
                 const std::vector<bool> &accepted);

  State state_;
  std::vector<int> status_;

  int landmarks_size_;

  static constexpr int REACHED = 0;
  static constexpr int NEEDED = 1;
  static constexpr int NOT_REACHED = 2;
};

} // namespace rwls

#endif // LANDMARK_COUNT_H_
