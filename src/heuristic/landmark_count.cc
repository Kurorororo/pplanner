#include "heuristic/landmark_count.h"

#include <limits>

using std::unordered_map;
using std::unordered_set;
using std::vector;

namespace rwls {

constexpr int LandmarkCount::REACHED;
constexpr int LandmarkCount::NEEDED;
constexpr int LandmarkCount::NOT_REACHED;

inline bool Get(int i, const uint8_t *set) {
  return (set[i / 8] & (1 << (i % 8))) != 0;
}

inline void Up(int i, uint8_t *set) {
  set[i / 8] |= 1 << (i % 8);
}

inline void Down(int i, uint8_t *set) {
  set[i / 8] &= ~(1 << (i % 8));
}

int LandmarkCount::ReachedSize(const State &state,
                               const uint8_t *parent_accepted,
                               uint8_t *accepted) {
  int size = 0;

  for (size_t psi_id=0, n=landmark_id_max; psi_id<n; ++psi_id) {
    if (Get(psi_id, parent_accepted)) {
      Up(psi_id, accepted);
      ++size;
      continue;
    }

    const Landmark &psi = graph.GetLandmark(psi_id);
    if (psi.IsEmpty() || !psi.IsImplicated(state)) continue;

    bool all = true;

    for (auto phi_id : graph.GetInitIdsByTermId(psi_id)) {
      if (!Get(phi_id, parent_accepted)) {
        all = false;
        break;
      }
    }
    if (all) {
      Up(psi_id, accepted);
      ++size;
    }
  }
  return size;
}

int LandmarkCount::NeededSize(const State &state, const Domain &domain,
                              const uint8_t *accepted) {
  size_t goal_size = domain.goal.size();
  int size = 0;

  for (size_t phi_id=0, n=landmark_id_max; phi_id<n; ++phi_id) {
    if (!Get(phi_id, accepted)) continue;
    status_[phi_id] = REACHED;

    const Landmark &phi = graph.GetLandmark(phi_id);
    if (phi.IsImplicated(state)) continue;

    if (phi_id < goal_size) {
      status_[phi_id] = NEEDED;
      ++size;
      continue;
    }

    for (auto psi_id : graph.GetTermIdsByInitId(phi_id)) {
      if (graph.GetOrderingType(phi_id, psi_id) == LandmarkGraph::GREEDY
          && !Get(psi_id, accepted)) {
        status_[phi_id] = NEEDED;
        ++size;
        break;
      }
    }
  }
  return size;
}

int LandmarkCount::operator()(const State &state, const Domain &domain,
                              const uint8_t *parent_accepted,
                              uint8_t *accepted) {
  if (GoalCheck(domain.goal, state)) return 0;

  std::fill(status_.begin(), status_.end(), NOT_REACHED);

  for (size_t i=0, n=accepted_bytes; i<n; ++i)
    accepted[i] = 0;

  int reached_size = 0;

  if (parent_accepted == nullptr) {
    for (size_t psi_id=0, n=landmark_id_max; psi_id<n; ++psi_id) {
      const Landmark &psi = graph.GetLandmark(psi_id);

      if (!psi.IsEmpty()
          && psi.IsImplicated(state)
          && graph.GetInitIdsByTermId(psi_id).empty()) {
        status_[psi_id] = REACHED;
        Up(psi_id, accepted);
        ++reached_size;
      }
    }
  } else {
    reached_size = ReachedSize(state, parent_accepted, accepted);
  }

  int needed_size = NeededSize(state, domain, accepted);

  for (size_t id=0, n=landmark_id_max; id<n; ++id) {
    if (graph.GetLandmark(id).IsEmpty()) continue;
    if ((status_[id] == NOT_REACHED && graph.GetFirstAchieversSize(id) == 0)
        || (status_[id] == NEEDED && graph.GetPossibleAchieversSize(id) == 0))
      return std::numeric_limits<int>::max();
  }

  int h = landmarks_size_ - reached_size + needed_size;

  return h;
}

int LandmarkCount::ReachedSize(const State &state,
                               const vector<bool> &parent_accepted,
                               vector<bool> &accepted) {
  int size = 0;
  for (size_t psi_id=0, n=landmark_id_max; psi_id<n; ++psi_id) {
    if (parent_accepted[psi_id]) {
      accepted[psi_id] = true;
      ++size;
      continue;
    }

    const Landmark &psi = graph.GetLandmark(psi_id);
    if (psi.IsEmpty() || !psi.IsImplicated(state)) continue;

    bool all = true;
    for (auto phi_id : graph.GetInitIdsByTermId(psi_id)) {
      if (!parent_accepted[phi_id]) {
        all = false;
        break;
      }
    }
    if (all) {
      accepted[psi_id] = true;
      ++size;
    }
  }
  return size;
}

int LandmarkCount::NeededSize(const State &state, const Domain &domain,
                              const vector<bool> &accepted) {
  size_t goal_size = domain.goal.size();
  int size = 0;
  for (size_t phi_id=0, n=landmark_id_max; phi_id<n; ++phi_id) {
    if (!accepted[phi_id]) continue;
    status_[phi_id] = REACHED;

    const Landmark &phi = graph.GetLandmark(phi_id);
    if (phi.IsImplicated(state)) continue;

    if (phi_id < goal_size) {
      status_[phi_id] = NEEDED;
      ++size;
      continue;
    }

    for (auto psi_id : graph.GetTermIdsByInitId(phi_id)) {
      if (graph.GetOrderingType(phi_id, psi_id) == LandmarkGraph::GREEDY
          && !accepted[psi_id]) {
        status_[phi_id] = NEEDED;
        ++size;
        break;
      }
    }
  }
  return size;
}

int LandmarkCount::operator()(const State &state, const Domain &domain,
                              const vector<bool> &parent_accepted,
                              vector<bool> &accepted, bool initial) {
  if (GoalCheck(domain.goal, state)) return 0;

  std::fill(status_.begin(), status_.end(), NOT_REACHED);
  std::fill(accepted.begin(), accepted.end(), false);
  int reached_size = 0;

  if (initial) {
    for (size_t psi_id=0, n=landmark_id_max; psi_id<n; ++psi_id) {
      const Landmark &psi = graph.GetLandmark(psi_id);
      if (!psi.IsEmpty()
          && psi.IsImplicated(state)
          && graph.GetInitIdsByTermId(psi_id).empty()) {
        status_[psi_id] = REACHED;
        accepted[psi_id] = true;
        ++reached_size;
      }
    }
  } else {
    reached_size = ReachedSize(state, parent_accepted, accepted);
  }

  int needed_size = NeededSize(state, domain, accepted);

  for (size_t id=0, n=landmark_id_max; id<n; ++id) {
    if (graph.GetLandmark(id).IsEmpty()) continue;
    if ((status_[id] == NOT_REACHED && graph.GetFirstAchieversSize(id) == 0)
        || (status_[id] == NEEDED && graph.GetPossibleAchieversSize(id) == 0))
      return std::numeric_limits<int>::max();
  }

  int h = landmarks_size_ - reached_size + needed_size;
  return h;
}

} // namespace rwls
