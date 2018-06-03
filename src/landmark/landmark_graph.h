#ifndef LANDMAK_GRAPH_H_
#define LANDMAK_GRAPH_H_

#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "landmark/landmark.h"

namespace pplanner {

class LandmarkGraph {
 public:
  enum OrderingType { GREEDY, NATURAL, REASONABLE, OBEDIENT, };

  LandmarkGraph() : orderings_size_(0) {}

  ~LandmarkGraph() {}

  const Landmark& GetLandmark(int id) const { return id_to_landmark_[id]; }

  Landmark CopyLandmark(int id) const { return id_to_landmark_[id]; }

  const std::vector<Landmark>& GetLandmarks() const { return id_to_landmark_; }

  std::vector<Landmark> CopyLandmarks() const { return id_to_landmark_; }

  size_t GetLandmarksSize() const { return landmark_to_id_.size(); }

  size_t GetOrderingsSize() const { return orderings_size_; }

  int ToId(const Landmark &landmark) const {
    return landmark_to_id_.at(landmark);
  }

  bool IsIn(const Landmark &landmark) const {
    return landmark_to_id_.find(landmark) != landmark_to_id_.end();
  }

  bool IsAdjacent(int init_id, int term_id) const {
    return adjacent_matrix_[init_id][term_id];
  }

  OrderingType GetOrderingType(int init_id, int term_id) const {
    return ordering_types_[init_id][term_id];
  }

  const std::vector<int>& GetTermIdsByInitId(int id) const {
    return init_id_to_term_ids_[id];
  }

  const std::vector<int>& GetInitIdsByTermId(int id) const {
    return term_id_to_init_ids_[id];
  }

  void AddOrdering(int init_id, int term_id, OrderingType type) {
    if (IsAdjacent(init_id, term_id)
        && type >= GetOrderingType(init_id, term_id)) return;
    ++orderings_size_;
    adjacent_matrix_[init_id][term_id] = true;
    ordering_types_[init_id][term_id] = type;
    init_id_to_term_ids_[init_id].push_back(term_id);
    term_id_to_init_ids_[term_id].push_back(init_id);
  }

  void DeleteOrdering(int init_id, int term_id);

  int Add(const Landmark &landmark);

  int Add(Landmark &&landmark);

  void Delete(int id);

  void Delete(const Landmark &landmark);

  void PushPossibleAchiever(int id, int action) {
    possible_achievers_[id].push_back(action);
  }

  const std::vector<int>& GetPossibleAchievers(int id) const {
    return possible_achievers_[id];
  }

  size_t GetPossibleAchieversSize(int id) const {
    return possible_achievers_[id].size();
  }

  void PushFirstAchiever(int id, int action) {
    first_achievers_[id].push_back(action);
  }

  const std::vector<int>& GetFirstAchievers(int id) const {
    return first_achievers_[id];
  }

  size_t GetFirstAchieversSize(int id) const {
    return first_achievers_[id].size();
  }

  void GetAncestors(int start, std::vector<int> &ancestors);

  int RemoveCycles(int start_id);

 private:
  void PrepareOrderings(int id);

  void RemoveEdge(int start);

  bool FindCycle(int start);

  size_t orderings_size_;

  std::unordered_map<Landmark, int, LandmarkHash> landmark_to_id_;
  std::vector<Landmark> id_to_landmark_;

  std::vector< std::vector<bool> > adjacent_matrix_;
  std::vector< std::vector<OrderingType> > ordering_types_;
  std::vector< std::vector<int> > init_id_to_term_ids_;
  std::vector< std::vector<int> > term_id_to_init_ids_;

  std::vector< std::vector<int> > possible_achievers_;
  std::vector< std::vector<int> > first_achievers_;

  std::stack<int> open_;
  std::unordered_set<int> closed_;
  std::vector<int> path_;
};

bool TypeCheck(const std::vector<LandmarkGraph::OrderingType> &allowd_types,
               LandmarkGraph::OrderingType type);

} // namespace pplanner

#endif // LANDMAK_GRAPH_H_
