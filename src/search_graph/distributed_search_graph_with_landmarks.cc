#include "search_graph/distributed_search_graph_with_landmarks.h"

namespace pplanner {

using std::vector;

uint8_t* DistributedSearchGraphWithLandmarks::Landmark(int i) {
  if (i < 0) {
    i *= -1;
    size_t size = static_cast<size_t>(i) * n_landmarks_bytes_;

    if (tmp_landmarks_.size() < size)
      tmp_landmarks_.resize(size);

    for (size_t j=0; j<n_landmarks_bytes_; ++j)
      tmp_landmarks_[size - n_landmarks_bytes_ + j] = 0;

    return tmp_landmarks_.data() + size - n_landmarks_bytes_;
  }

  return landmarks_.data() + static_cast<size_t>(i) * n_landmarks_bytes_;
}

uint8_t* DistributedSearchGraphWithLandmarks::ParentLandmark(int i) {
  int parent_rank = ParentRank(i);

  if (parent_rank == -1) return nullptr;

  if (parent_rank == rank()) return Landmark(Parent(i));

  memcpy(parent_landmark_.data(), Landmark(i),
         n_landmarks_bytes_ * sizeof(uint8_t));

  return parent_landmark_.data();
}

void DistributedSearchGraphWithLandmarks::BufferNode(int i,
                                                     unsigned char *buffer) {
  DistributedSearchGraph::BufferNode(i, buffer);
  uint8_t *landmark = reinterpret_cast<uint8_t*>(
      buffer + DistributedSearchGraph::node_size());
  memcpy(landmark, Landmark(i), n_landmarks_bytes_ * sizeof(uint8_t));
}

void DistributedSearchGraphWithLandmarks::BufferNode(int action,
                                                     int parent_node,
                                                     const vector<int> &parent,
                                                     const vector<int> &state,
                                                     unsigned char *buffer) {
  DistributedSearchGraph::BufferNode(
      action, parent_node, parent, state, buffer);
  uint8_t *landmark = reinterpret_cast<uint8_t*>(
      buffer + DistributedSearchGraph::node_size());
  memcpy(landmark, Landmark(parent_node), n_landmarks_bytes_ * sizeof(uint8_t));
}

void DistributedSearchGraphWithLandmarks::BufferNode(int action,
                                                     int parent_node,
                                                     const vector<int> &state,
                                                     unsigned char *buffer) {
  DistributedSearchGraph::BufferNode(action, parent_node, state, buffer);
  uint8_t *landmark = reinterpret_cast<uint8_t*>(
      buffer + DistributedSearchGraph::node_size());
  memcpy(landmark, Landmark(parent_node), n_landmarks_bytes_ * sizeof(uint8_t));
}

void DistributedSearchGraphWithLandmarks::BufferNode(int action,
                                                     int parent_node,
                                                     uint32_t hash_value,
                                                     const uint32_t *packed,
                                                     unsigned char *buffer) {
  DistributedSearchGraph::BufferNode(
      action, parent_node, hash_value, packed, buffer);
  uint8_t *landmark = reinterpret_cast<uint8_t*>(
      buffer + DistributedSearchGraph::node_size());
  memcpy(landmark, Landmark(parent_node), n_landmarks_bytes_ * sizeof(uint8_t));
}

int DistributedSearchGraphWithLandmarks::GenerateNodeIfNotClosedFromBytes(
    const unsigned char *d) {
  int node = DistributedSearchGraph::GenerateNodeIfNotClosedFromBytes(d);

  if (node != -1) {
    const uint8_t *landmark = reinterpret_cast<const uint8_t*>(
        d + DistributedSearchGraph::node_size());
    size_t index = landmarks_.size() - n_landmarks_bytes_;
    memcpy(landmarks_.data() + index, landmark,
           n_landmarks_bytes_ * sizeof(uint8_t));
  }

  return node;
}

int DistributedSearchGraphWithLandmarks::GenerateAndCloseNodeFromBytes(
    const unsigned char *d) {
  int node = DistributedSearchGraph::GenerateAndCloseNodeFromBytes(d);

  if (node != -1) {
    const uint8_t *landmark = reinterpret_cast<const uint8_t*>(
        d + DistributedSearchGraph::node_size());
    size_t index = landmarks_.size() - n_landmarks_bytes_;
    memcpy(landmarks_.data() + index, landmark,
           n_landmarks_bytes_ * sizeof(uint8_t));
  }

  return node;
}

int DistributedSearchGraphWithLandmarks::GenerateNodeFromBytes(
    const unsigned char *d,
    vector<int> &values) {
  int node = DistributedSearchGraph::GenerateNodeFromBytes(d, values);
  const uint8_t *landmark = reinterpret_cast<const uint8_t*>(
      d + n_evaluators() * sizeof(int) + DistributedSearchGraph::node_size());
  size_t index = landmarks_.size() - n_landmarks_bytes_;
  memcpy(landmarks_.data() + index, landmark,
         n_landmarks_bytes_ * sizeof(uint8_t));

  return node;
}

int DistributedSearchGraphWithLandmarks::GenerateEvaluatedNode(
    int index,
    int action,
    int parent_node,
    uint32_t hash_value,
    const uint32_t *packed,
    int parent_rank) {
  int node = GenerateNode(action, parent_node, hash_value, packed, parent_rank);
  const uint8_t *tmp = tmp_landmarks_.data()
    + static_cast<size_t>(index) * n_landmarks_bytes_;
  memcpy(Landmark(node), tmp, n_landmarks_bytes_ * sizeof(uint8_t));

  return node;
}

void DistributedSearchGraphWithLandmarks::BufferEvaluatedNode(
    int index,
    int action,
    int parent_node,
    uint32_t hash_value,
    const uint32_t *packed,
    unsigned char *buffer) {
  DistributedSearchGraph::BufferNode(
      action, parent_node, hash_value, packed, buffer);
  uint8_t *landmark = reinterpret_cast<uint8_t*>(
      buffer + DistributedSearchGraph::node_size());
  const uint8_t *tmp = tmp_landmarks_.data()
    + static_cast<size_t>(index) * n_landmarks_bytes_;
  memcpy(landmark, tmp, n_landmarks_bytes_ * sizeof(uint8_t));
}

} // namespace pplanner
