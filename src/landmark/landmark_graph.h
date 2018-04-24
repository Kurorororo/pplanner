#ifndef LANDMAK_GRAPH_H_
#define LANDMAK_GRAPH_H_

#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "landmark/landmark.h"

namespace rwls {

class LandmarkGraph {
 public:
  LandmarkGraph() {
    orderings_size_ = 0;
  }

  ~LandmarkGraph() {}

  inline const Landmark& GetLandmark(size_t id) const {
    return id_to_landmark_[id];
  }

  inline Landmark CopyLandmark(size_t id) const {
    return id_to_landmark_[id];
  }

  inline const std::vector<Landmark>& GetLandmarks() const {
    return id_to_landmark_;
  }

  inline std::vector<Landmark> CopyLandmarks() const {
    return id_to_landmark_;
  }

  inline size_t GetLandmarksSize() const {
    return landmark_to_id_.size();
  }

  inline size_t GetOrderingsSize() const {
    return orderings_size_;
  }

  inline size_t ToId(const Landmark &landmark) const {
    return landmark_to_id_.at(landmark);
  }

  inline bool IsIn(const Landmark &landmark) const {
    return landmark_to_id_.find(landmark) != landmark_to_id_.end();
  }

  inline bool IsAdjacent(size_t init_id, size_t term_id) const {
    return adjacent_matrix_[init_id][term_id];
  }

  inline int GetOrderingType(size_t init_id, size_t term_id) const {
    return ordering_types_[init_id][term_id];
  }

  inline const std::vector<size_t>& GetTermIdsByInitId(size_t id) const {
    return init_id_to_term_ids_[id];
  }

  inline const std::vector<size_t>& GetInitIdsByTermId(size_t id) const {
    return term_id_to_init_ids_[id];
  }

  inline void AddOrdering(size_t init_id, size_t term_id, int type) {
    if (IsAdjacent(init_id, term_id)
        && type >= GetOrderingType(init_id, term_id)) return;
    ++orderings_size_;
    adjacent_matrix_[init_id][term_id] = true;
    ordering_types_[init_id][term_id] = type;
    init_id_to_term_ids_[init_id].push_back(term_id);
    term_id_to_init_ids_[term_id].push_back(init_id);
  }

  void DeleteOrdering(size_t init_id, size_t term_id);

  size_t Add(const Landmark &landmark);

  size_t Add(Landmark &&landmark);

  void Delete(size_t id);

  void Delete(const Landmark &landmark);

  void PushPossibleAchiever(size_t id, int action) {
    possible_achievers_[id].push_back(action);
  }

  const std::vector<int>& GetPossibleAchievers(size_t id) const {
    return possible_achievers_[id];
  }

  size_t GetPossibleAchieversSize(size_t id) const {
    return possible_achievers_[id].size();
  }

  void PushFirstAchiever(size_t id, int action) {
    first_achievers_[id].push_back(action);
  }

  const std::vector<int>& GetFirstAchievers(size_t id) const {
    return first_achievers_[id];
  }

  size_t GetFirstAchieversSize(size_t id) const {
    return first_achievers_[id].size();
  }

  void GetAncestors(size_t start, std::vector<size_t> &ancestors);

  int RemoveCycles(size_t start_id);

  static constexpr int GREEDY = 0;
  static constexpr int NATURAL = 1;
  static constexpr int REASONABLE = 2;
  static constexpr int OBEDIENT = 3;

 private:
  void PrepareOrderings(size_t id);

  void RemoveEdge(size_t start);

  bool FindCycle(size_t start);

  size_t orderings_size_;

  std::unordered_map<Landmark, size_t, LandmarkHash> landmark_to_id_;
  std::vector<Landmark> id_to_landmark_;

  std::vector< std::vector<bool> > adjacent_matrix_;
  std::vector< std::vector<int> > ordering_types_;
  std::vector< std::vector<size_t> > init_id_to_term_ids_;
  std::vector< std::vector<size_t> > term_id_to_init_ids_;

  std::vector< std::vector<int> > possible_achievers_;
  std::vector< std::vector<int> > first_achievers_;

  std::stack<size_t> open_;
  std::unordered_set<size_t> closed_;
  std::vector<size_t> path_;

};

bool TypeCheck(const std::vector<int> &allowd_types, int type);

} // namespace rwls

#endif // LANDMAK_GRAPH_H_
