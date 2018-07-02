#include "search_graph/distributed_search_graph_with_landmarks.h"

namespace pplanner {

using std::vector;

uint8_t* DistributedSearchGraphWithLandmarks::ParentLandmark(int i) {
  int parent_rank= ParentRank(i);

  if (parent_rank == -1) return nullptr;

  if (parent_rank == rank()) return Landmark(Parent(i));

  memcpy(parent_landmark_.data(), Landmark(i),
         n_landmarks_bytes_ * sizeof(uint8_t));

  return parent_landmark_.data();
}

void DistributedSearchGraphWithLandmarks::BufferNode(int parent, int action,
                                                     const vector<int> &state,
                                                     unsigned char *buffer) {
  DistributedSearchGraph::BufferNode(parent, action, state, buffer);
  uint8_t *landmark = reinterpret_cast<uint8_t*>(
      buffer + SearchGraph::NodeSize());
  memcpy(landmark, Landmark(parent), n_landmarks_bytes_ * sizeof(uint8_t));
}

int DistributedSearchGraphWithLandmarks::GenerateNodeIfNotClosed(
    const unsigned char *d) {
  int node = DistributedSearchGraph::GenerateNodeIfNotClosed(d);

  if (node != -1) {
    const uint8_t *landmark = reinterpret_cast<const uint8_t*>(
        d + SearchGraph::NodeSize());
    size_t index = landmarks_.size();
    landmarks_.resize(index + n_landmarks_bytes_, 0);
    memcpy(landmarks_.data() + index, landmark,
           n_landmarks_bytes_ * sizeof(uint8_t));
  }

  return node;
}

int DistributedSearchGraphWithLandmarks::GenerateNode(const unsigned char *d,
                                                      int *h) {
  int node = DistributedSearchGraph::GenerateNode(d, h);
  const uint8_t *landmark = reinterpret_cast<const uint8_t*>(
      d + sizeof(int) + SearchGraph::NodeSize());
  size_t index = landmarks_.size();
  landmarks_.resize(index + n_landmarks_bytes_, 0);
  memcpy(landmarks_.data() + index, landmark,
         n_landmarks_bytes_ * sizeof(uint8_t));

  return node;
}

} // namespace pplanner
