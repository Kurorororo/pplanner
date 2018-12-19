#ifndef SEARCH_GRAPH_WITH_LANDMARKS_H_
#define SEARCH_GRAPH_WITH_LANDMARKS_H_

#include <memory>
#include <vector>

#include "search_graph.h"
#include "landmark/landmark_graph.h"

namespace pplanner {

class SearchGraphWithLandmarks : public SearchGraph {
 public:
  SearchGraphWithLandmarks(std::shared_ptr<const SASPlus> problem,
                           int closed_exponent)
    : SearchGraph(problem, closed_exponent), n_landmarks_bytes_(0) {}

  ~SearchGraphWithLandmarks() {}

  void InitLandmarks(std::shared_ptr<const LandmarkGraph> graph) override {
    n_landmarks_bytes_ = (graph->landmark_id_max() + 7) / 8;
  }

  virtual std::size_t node_size() const override {
    return n_landmarks_bytes_ * sizeof(uint8_t) + SearchGraph::node_size();
  }

  virtual void Reserve(std::size_t size) override {
    SearchGraph::Reserve(size);
    landmarks_.reserve(n_landmarks_bytes_ * size);
  }

  virtual void AddProperties(int action, int parent_node, uint32_t hash_value)
    override {
    SearchGraph::AddProperties(action, parent_node, hash_value);
    landmarks_.resize(landmarks_.size() + n_landmarks_bytes_, 0);
  }

  uint8_t* Landmark(int i) override {
    return landmarks_.data() + i * n_landmarks_bytes_;
  }

  uint8_t* ParentLandmark(int i) override {
    int parent = Parent(i);

    if (parent == -1) return nullptr;

    return landmarks_.data() + parent * n_landmarks_bytes_;
  }

 private:
  std::size_t n_landmarks_bytes_;
  std::vector<uint8_t> landmarks_;
};

} // namespace pplanner

#endif // SEARCH_GRAPH_WITH_LANDMARKS_H_
