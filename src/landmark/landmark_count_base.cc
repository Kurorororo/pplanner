#include "landmark/landmark_count_base.h"

#include "utils/bit_vector.h"

namespace pplanner {

using std::vector;

int LandmarkCountBase::ReachedSize(const vector<int> &state,
                                   const uint8_t *parent_accepted,
                                   uint8_t *accepted) {
  int size = 0;

  for (int psi_id=0, n=landmark_graph_->landmark_id_max(); psi_id<n; ++psi_id) {
    if (Get(parent_accepted, psi_id)) {
      Up(accepted, psi_id);
      ++size;
      continue;
    }

    const Landmark &psi = landmark_graph_->GetLandmark(psi_id);
    if (psi.IsEmpty() || !psi.IsImplicated(state)) continue;

    bool all = true;

    for (auto phi_id : landmark_graph_->GetInitIdsByTermId(psi_id)) {
      if (!Get(parent_accepted, phi_id)) {
        all = false;
        break;
      }
    }
    if (all) {
      Up(accepted, psi_id);
      ++size;
    }
  }

  return size;
}

int LandmarkCountBase::NeededSize(const vector<int> &state,
                                  const uint8_t *accepted) {
  int goal_size = problem_->n_goal_facts();
  int size = 0;

  for (int phi_id=0, n=landmark_graph_->landmark_id_max(); phi_id<n; ++phi_id) {
    if (!Get(accepted, phi_id)) continue;
    status_[phi_id] = REACHED;

    const Landmark &phi = landmark_graph_->GetLandmark(phi_id);
    if (phi.IsImplicated(state)) continue;

    if (phi_id < goal_size) {
      status_[phi_id] = NEEDED;
      ++size;
      continue;
    }

    for (auto psi_id : landmark_graph_->GetTermIdsByInitId(phi_id)) {
      auto ordering_type = landmark_graph_->GetOrderingType(phi_id, psi_id);

      if (ordering_type == LandmarkGraph::GREEDY && !Get(accepted, psi_id)) {
        status_[phi_id] = NEEDED;
        ++size;
        break;
      }
    }
  }

  return size;
}

int LandmarkCountBase::Evaluate(const vector<int> &state,
                                const uint8_t *parent_accepted,
                                uint8_t *accepted) {
  if (problem_->IsGoal(state)) return 0;

  std::fill(status_.begin(), status_.end(), NOT_REACHED);
  int reached_size = 0;

  if (parent_accepted == nullptr) {
    for (int i=0, n=landmark_graph_->landmark_id_max(); i<n; ++i) {
      const Landmark &psi = landmark_graph_->GetLandmark(i);

      if (!psi.IsEmpty()
          && psi.IsImplicated(state)
          && landmark_graph_->GetInitIdsByTermId(i).empty()) {
        status_[i] = REACHED;
        Up(accepted, i);
        ++reached_size;
      }
    }
  } else {
    reached_size = ReachedSize(state, parent_accepted, accepted);
  }

  int needed_size = NeededSize(state, accepted);

  for (int i=0, n=landmark_graph_->landmark_id_max(); i<n; ++i) {
    const Landmark &psi = landmark_graph_->GetLandmark(i);

    if (!psi.IsEmpty() && ((status_[i] == NOT_REACHED
          && landmark_graph_->GetFirstAchieversSize(i) == 0)
        || (status_[i] == NEEDED
          && landmark_graph_->GetPossibleAchieversSize(i) == 0)))
      return -1;
  }

  return landmark_graph_->n_landmarks() - reached_size + needed_size;
}

} // namespace pplanner
