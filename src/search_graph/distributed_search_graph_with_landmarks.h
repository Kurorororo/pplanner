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
  DistributedSearchGraphWithLandmarks(std::shared_ptr<const SASPlus> problem,
                                      int closed_exponent, int n_evaluators,
                                      int rank)
    : DistributedSearchGraph(problem, closed_exponent, n_evaluators, rank),
      n_landmarks_bytes_(0) {}

  virtual ~DistributedSearchGraphWithLandmarks() {}

  void InitLandmarks(std::shared_ptr<const LandmarkGraph> graph) override {
    n_landmarks_bytes_ = (graph->landmark_id_max() + 7) / 8;
    parent_landmark_.resize(n_landmarks_bytes_, 0);
  }

  virtual size_t node_size() const override {
    int node_size = DistributedSearchGraph::node_size();

    return n_landmarks_bytes_ * sizeof(uint8_t) + node_size;
  }

  virtual void Reserve(size_t size) override {
    DistributedSearchGraph::Reserve(size);
    landmarks_.reserve(n_landmarks_bytes_ * size);
  }

  virtual void AddMoreProperties(int parent_rank) override {
    DistributedSearchGraph::AddMoreProperties(parent_rank);
    landmarks_.resize(landmarks_.size() + n_landmarks_bytes_, 0);
  }

  virtual int GenerateNodeIfNotClosed(const unsigned char *d) override;

  virtual int GenerateNode(const unsigned char *d, std::vector<int> &values)
    override;

  virtual void BufferNode(int action, int parent_node,
                          const std::vector<int> &parent,
                          const std::vector<int> &state,
                          unsigned char *buffer) override;

  uint8_t* Landmark(int i) override {
    return landmarks_.data() + static_cast<size_t>(i) * n_landmarks_bytes_;
  }

  uint8_t* ParentLandmark(int i) override;

 private:
  size_t n_landmarks_bytes_;
  std::vector<uint8_t> parent_landmark_;
  std::vector<uint8_t> landmarks_;
};

} // namespace pplanner

#endif // DISTRIBUTED_SEARCH_GRAPH_WITH_LANDMARKS_H_
