#ifndef DISTRIBUTED_SEARCH_GRAPH_WITH_LANDMARKS_H_
#define DISTRIBUTED_SEARCH_GRAPH_WITH_LANDMARKS_H_

#include <memory>
#include <vector>

#include "sas_plus.h"
#include "search_graph/distributed_search_graph.h"
#include "landmark/landmark_graph.h"

namespace pplanner {

class DistributedSearchGraphWithLandmarks : public DistributedSearchGraph {
 public:
  DistributedSearchGraphWithLandmarks() : DistributedSearchGraph(),
                                          n_landmarks_bytes_(0) {}

  DistributedSearchGraphWithLandmarks(const SASPlus &problem,
                                      int closed_exponent, int rank)
    : DistributedSearchGraph(problem, closed_exponent, rank),
      n_landmarks_bytes_(0) {}

  virtual ~DistributedSearchGraphWithLandmarks() {}

  void InitLandmarks(std::shared_ptr<const LandmarkGraph> graph) override {
    n_landmarks_bytes_ = (graph->landmark_id_max() + 7) / 8;
    parent_landmark_.resize(n_landmarks_bytes_, 0);
  }

  virtual size_t NodeSize() const override {
    int node_size = DistributedSearchGraph::NodeSize();

    return n_landmarks_bytes_ * sizeof(uint8_t) + node_size;
  }

  virtual void Reserve(size_t size) override {
    DistributedSearchGraph::Reserve(size);
    landmarks_.reserve(n_landmarks_bytes_ * size);
  }

  virtual int GenerateNode(const std::vector<int> &state, int parent,
                           int action, bool is_preferred, int parent_rank)
    override {
    int node = DistributedSearchGraph::GenerateNode(
        state, parent, action, is_preferred, parent_rank);
    landmarks_.resize(landmarks_.size() + n_landmarks_bytes_, 0);

    return node;
  }

  virtual int GenerateNode(const uint32_t *packed, int parent, int action,
                           bool is_preferred, int parent_rank) override {
    int node = DistributedSearchGraph::GenerateNode(
        packed, parent, action, is_preferred, parent_rank);
    landmarks_.resize(landmarks_.size() + n_landmarks_bytes_, 0);

    return node;
  }

  virtual int GenerateNodeIfNotClosed(const std::vector<int> &state, int parent,
                                      int action, bool is_preferred,
                                      int parent_rank) override {
    int node = DistributedSearchGraph::GenerateNodeIfNotClosed(
        state, parent, action, is_preferred, parent_rank);

    if (node != -1)
      landmarks_.resize(landmarks_.size() + n_landmarks_bytes_, 0);

    return node;
  }

  virtual int GenerateNodeIfNotClosed(const uint32_t *packed, int parent,
                                      int action, bool is_preferred,
                                      int parent_rank) override {
    int node = DistributedSearchGraph::GenerateNodeIfNotClosed(
        packed, parent, action, is_preferred, parent_rank);

    if (node != -1)
      landmarks_.resize(landmarks_.size() + n_landmarks_bytes_, 0);

    return node;
  }

  virtual int GenerateNodeIfNotClosed(const unsigned char *d) override;

  virtual int GenerateNode(const unsigned char *d, int *h) override;

  virtual void BufferNode(int parent, int action, const std::vector<int> &state,
                          unsigned char *buffer) override;

  uint8_t* Landmark(int i) override {
    return landmarks_.data() + i * n_landmarks_bytes_;
  }

  uint8_t* ParentLandmark(int i) override;

 private:
  size_t n_landmarks_bytes_;
  std::vector<uint8_t> parent_landmark_;
  std::vector<uint8_t> landmarks_;
};

} // namespace pplanner

#endif // DISTRIBUTED_SEARCH_GRAPH_WITH_LANDMARKS_H_
