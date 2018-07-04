#ifndef SEARCH_GRAPH_WITH_LANDMARKS_H_
#define SEARCH_GRAPH_WITH_LANDMARKS_H_

#include <memory>
#include <vector>

#include "search_graph.h"
#include "landmark/landmark_graph.h"

namespace pplanner {

class SearchGraphWithLandmarks : public SearchGraph {
 public:
  SearchGraphWithLandmarks() : SearchGraph(), n_landmarks_bytes_(0) {}

  SearchGraphWithLandmarks(std::shared_ptr<const SASPlus> problem,
                           int closed_exponent)
    : SearchGraph(problem, closed_exponent), n_landmarks_bytes_(0) {}

  ~SearchGraphWithLandmarks() {}

  void InitLandmarks(std::shared_ptr<const LandmarkGraph> graph) override {
    n_landmarks_bytes_ = (graph->landmark_id_max() + 7) / 8;
  }

  virtual size_t NodeSize() const override {
    return n_landmarks_bytes_ * sizeof(uint8_t) + SearchGraph::NodeSize();
  }

  virtual void Reserve(size_t size) override {
    SearchGraph::Reserve(size);
    landmarks_.reserve(n_landmarks_bytes_ * size);
  }

  virtual int GenerateNode(const std::vector<int> &state, int parent,
                           int action, bool is_preferred) override {
    int node = SearchGraph::GenerateNode(state, parent, action, is_preferred);
    landmarks_.resize(landmarks_.size() + n_landmarks_bytes_, 0);

    return node;
  }

  virtual int GenerateNodeIfNotClosed(const std::vector<int> &state, int parent,
                                      int action, bool is_preferred) override {
    int node = SearchGraph::GenerateNodeIfNotClosed(
        state, parent, action, is_preferred);

    if (node != -1) {
      landmarks_.resize(landmarks_.size() + n_landmarks_bytes_, 0);
    }

    return node;
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
  size_t n_landmarks_bytes_;
  std::vector<uint8_t> landmarks_;
};

} // namespace pplanner

#endif // SEARCH_GRAPH_WITH_LANDMARKS_H_
