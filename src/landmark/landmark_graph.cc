#include "landmark/landmark_graph.h"

#include <algorithm>
#include <stack>

namespace pplanner {

using std::find;
using std::remove_if;
using std::stack;
using std::unordered_map;
using std::unordered_set;
using std::vector;

void LandmarkGraph::DeleteOrdering(int init_id, int term_id) {
  --n_orderings_;

  adjacent_matrix_[init_id][term_id] = false;

  auto &term_ids = init_id_to_term_ids_[init_id];
  auto result = remove_if(term_ids.begin(), term_ids.end(),
                          [term_id](int id) { return id == term_id; });
  term_ids.erase(result, term_ids.end());

  auto &init_ids = term_id_to_init_ids_[term_id];
  result = remove_if(init_ids.begin(), init_ids.end(),
                     [init_id](int id) { return id == init_id; });
  init_ids.erase(result, init_ids.end());
}

void LandmarkGraph::PrepareOrderings(int id) {
  init_id_to_term_ids_.resize(id + 1);
  term_id_to_init_ids_.resize(id + 1);
  adjacent_matrix_.resize(id + 1);
  ordering_types_.resize(id + 1);

  for (auto &row : adjacent_matrix_)
    row.resize(id + 1, false);

  for (auto &row : ordering_types_)
    row.resize(id + 1);

  first_achievers_.resize(id + 1);
  possible_achievers_.resize(id + 1);
}

bool LandmarkGraph::RemoveDisjunctive(const Landmark &landmark) {
  // If fact landmark, remove a disjunctive landmark dominated by the fact.
  if (!landmark.IsFact()) return false;

  int chi_id = FactToId(landmark.VarValue(0));

  if (chi_id == -1) return false;

  const Landmark &chi = GetLandmark(chi_id);

  if (landmark == chi) return false;

  Delete(chi_id);

  return true;
}

int LandmarkGraph::Add(const Landmark &landmark) {
  bool removed = RemoveDisjunctive(landmark);

  /** If no disjunctive landmark is removed, there may be overlapping landmarks.
   *  If any, the landmark is not added to the graph.
   *  A fact must not appear in more than one landmarks.
   **/
  if (!removed) {
    for (int i=0, n=landmark.size(); i<n; ++i) {
      int chi_id = FactToId(landmark.VarValue(i));

      if (chi_id != -1) return -1;
    }
  }

  int id = id_to_landmark_.size();
  id_to_landmark_.push_back(landmark);
  PrepareOrderings(id);

  for (int i=0, n=landmark.size(); i<n; ++i) {
    int f = problem_->Fact(landmark.VarValue(i));
    fact_to_id_[f] = id;
  }

  ++n_landmarks_;
  if (landmark.size() > 1) ++n_disjunctive_;

  return id;
}

void LandmarkGraph::Delete(int id) {
  n_orderings_ -= init_id_to_term_ids_[id].size();
  n_orderings_ -= term_id_to_init_ids_[id].size();

  for (auto term_id : init_id_to_term_ids_[id])
    adjacent_matrix_[id][term_id] = false;

  for (auto init_id : term_id_to_init_ids_[id])
    adjacent_matrix_[init_id][id] = false;

  auto condition = [id](int x) { return x == id; };

  for (auto term_id : init_id_to_term_ids_[id]) {
    auto &ids = term_id_to_init_ids_[term_id];
    auto result = remove_if(ids.begin(), ids.end(), condition);
    ids.erase(result, ids.end());
  }

  for (auto init_id : term_id_to_init_ids_[id]) {
    auto &ids = init_id_to_term_ids_[init_id];
    auto result = remove_if(ids.begin(), ids.end(), condition);
    ids.erase(result, ids.end());
  }

  auto landmark = id_to_landmark_[id];

  for (int i=0, n=landmark.size(); i<n; ++i) {
    int f = problem_->Fact(landmark.VarValue(i));
    fact_to_id_[f] = -1;
  }

  if (landmark.size() > 1) --n_disjunctive_;

  init_id_to_term_ids_[id].clear();
  term_id_to_init_ids_[id].clear();

  possible_achievers_[id].clear();
  first_achievers_[id].clear();

  id_to_landmark_[id].Clear();

  --n_landmarks_;
}

void LandmarkGraph::Delete(const Landmark &landmark) {
  int id = ToId(landmark);

  if (id == -1) return;

  Delete(id);
}

void LandmarkGraph::GetAncestors(int start, vector<int> &ancestors) const {
  static stack<int> open;
  static unordered_set<int> closed;

  ancestors.clear();
  closed.clear();
  open = stack<int>();

  open.push(start);
  closed.insert(start);

  while (!open.empty()) {
    int current = open.top();
    open.pop();

    for (auto parent : GetInitIdsByTermId(current)) {
      if (closed.find(parent) != closed.end()) continue;
      closed.insert(parent);
      ancestors.push_back(parent);
      open.push(parent);
    }
  }
}

void LandmarkGraph::RemoveEdge(int start, vector<int> &path) {
  int parent = path.back();
  auto start_it = find(path.begin(), path.end(), start);

  for (auto it = start_it; it != path.end(); ++it) {
    int child = *it;

    if (GetOrderingType(parent, child) == OBEDIENT) {
      DeleteOrdering(parent, child);

      return;
    }

    parent = child;
  }

  parent = path.back();

  for (auto it = start_it; it != path.end(); ++it) {
    int child = *it;

    if (GetOrderingType(parent, child) == REASONABLE) {
      DeleteOrdering(parent, child);

      return;
    }

    parent = child;
  }
}

bool LandmarkGraph::FindCycle(int current, vector<int> &path,
                              unordered_set<int> &closed) {
  if (closed.find(current) != closed.end()) {
    RemoveEdge(current, path);

    return true;
  }

  path.push_back(current);
  closed.insert(current);

  for (auto child : GetTermIdsByInitId(current))
    if (FindCycle(child, path, closed)) return true;

  path.pop_back();
  closed.erase(current);

  return false;
}

int LandmarkGraph::RemoveCycles(int start) {
  static unordered_set<int> closed;
  static vector<int> path;

  int cycles = -1;

  do {
    path.clear();
    closed.clear();
    ++cycles;
  } while (FindCycle(start, path, closed));

  return cycles;
}

bool TypeCheck(const vector<LandmarkGraph::OrderingType> &allowed_types,
               LandmarkGraph::OrderingType type) {
  auto result = find(allowed_types.begin(), allowed_types.end(), type);

  return result != allowed_types.end();
}

} //namespace pplanner
