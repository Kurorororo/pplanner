#ifndef LANDMAK_GRAPH_H_
#define LANDMAK_GRAPH_H_

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "sas_plus.h"
#include "landmark/landmark.h"

namespace pplanner {

class LandmarkGraph {
 public:
  enum OrderingType { GREEDY, NATURAL, REASONABLE, OBEDIENT, };

  LandmarkGraph() : n_landmarks_(0), n_disjunctive_(0), n_orderings_(0),
                    problem_(nullptr) {}

  LandmarkGraph(std::shared_ptr<const SASPlus> problem)
    : n_landmarks_(0),
      n_disjunctive_(0),
      n_orderings_(0),
      fact_to_id_(problem->n_facts(), -1),
      problem_(problem) {}

  ~LandmarkGraph() {}

  size_t landmark_id_max() const { return id_to_landmark_.size(); }

  size_t n_landmarks() const { return n_landmarks_; }

  size_t n_disjunctive() const { return n_disjunctive_; }

  size_t n_orderings() const { return n_orderings_; }

  const Landmark& GetLandmark(int id) const { return id_to_landmark_[id]; }

  Landmark CopyLandmark(int id) const { return id_to_landmark_[id]; }

  const std::vector<Landmark>& GetLandmarks() const { return id_to_landmark_; }

  std::vector<Landmark> CopyLandmarks() const { return id_to_landmark_; }

  int FactToId(int f) const { return fact_to_id_[f]; }

  int ToId(const Landmark &landmark) const {
    int f = problem_->Fact(landmark.VarValue(0));
    int id = FactToId(f);

    if (id == -1 || landmark != GetLandmark(id)) return -1;

    return id;
  }

  bool IsIn(const Landmark &landmark) const {
    int id = ToId(landmark);

    return id != -1 && landmark == GetLandmark(id);
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
    ++n_orderings_;
    adjacent_matrix_[init_id][term_id] = true;
    ordering_types_[init_id][term_id] = type;
    init_id_to_term_ids_[init_id].push_back(term_id);
    term_id_to_init_ids_[term_id].push_back(init_id);
  }

  void DeleteOrdering(int init_id, int term_id);

  int Add(const Landmark &landmark);

  void Delete(int id);

  void Delete(const Landmark &landmark);

  int FactToId(const std::pair<int, int> &p) const {
    return FactToId(problem_->Fact(p));
  }

  bool IsGoal(int id) const {
    return id < static_cast<int>(problem_->n_goal_facts());
  }

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

  void GetAncestors(int start, std::vector<int> &ancestors) const;

  int RemoveCycles(int start_id);

 private:
  void PrepareOrderings(int id);

  bool RemoveDisjunctive(const Landmark &landmark);

  void RemoveEdge(int start, std::vector<int> &path);

  bool FindCycle(int start, std::vector<int> &path,
                 std::unordered_set<int> &cloesd);

  size_t n_landmarks_;
  size_t n_disjunctive_;
  size_t n_orderings_;

  // A fact must not appear in more than one landmarks.
  std::vector<int> fact_to_id_;
  std::vector<Landmark> id_to_landmark_;

  std::vector< std::vector<bool> > adjacent_matrix_;
  std::vector< std::vector<OrderingType> > ordering_types_;
  std::vector< std::vector<int> > init_id_to_term_ids_;
  std::vector< std::vector<int> > term_id_to_init_ids_;

  std::vector< std::vector<int> > possible_achievers_;
  std::vector< std::vector<int> > first_achievers_;

  std::shared_ptr<const SASPlus> problem_;
};

bool TypeCheck(const std::vector<LandmarkGraph::OrderingType> &allowd_types,
               LandmarkGraph::OrderingType type);

} // namespace pplanner

#endif // LANDMAK_GRAPH_H_
