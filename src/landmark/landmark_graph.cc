#include "landmark/landmark_graph.h"

#include <algorithm>

using std::find;
using std::remove_if;
using std::stack;
using std::unordered_map;
using std::unordered_set;
using std::vector;

namespace rwls {

void LandmarkGraph::DeleteOrdering(size_t init_id, size_t term_id) {
  --orderings_size_;

  adjacent_matrix_[init_id][term_id] = false;

  auto &term_ids = init_id_to_term_ids_[init_id];
  auto result = remove_if(term_ids.begin(), term_ids.end(),
                          [term_id](size_t id) { return id == term_id; });
  term_ids.erase(result, term_ids.end());

  auto &init_ids = term_id_to_init_ids_[term_id];
  result = remove_if(init_ids.begin(), init_ids.end(),
                     [init_id](size_t id) { return id == init_id; });
  init_ids.erase(result, init_ids.end());
}

void LandmarkGraph::PrepareOrderings(size_t id) {
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

size_t LandmarkGraph::Add(const Landmark &landmark) {
  size_t id = id_to_landmark_.size();
  landmark_to_id_[landmark] = id;
  id_to_landmark_.push_back(landmark);
  PrepareOrderings(id);
  return id;
}

size_t LandmarkGraph::Add(Landmark &&landmark) {
  size_t id = id_to_landmark_.size();
  landmark_to_id_[landmark] = id;
  id_to_landmark_.push_back(std::move(landmark));
  PrepareOrderings(id);
  return id;
}

void LandmarkGraph::Delete(size_t id) {
  orderings_size_ -= init_id_to_term_ids_[id].size();
  orderings_size_ -= term_id_to_init_ids_[id].size();

  for (auto term_id : init_id_to_term_ids_[id])
    adjacent_matrix_[id][term_id] = false;
  for (auto init_id : term_id_to_init_ids_[id])
    adjacent_matrix_[init_id][id] = false;

  auto condition = [id](size_t x) { return x == id; };
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

  init_id_to_term_ids_[id].clear();
  term_id_to_init_ids_[id].clear();

  possible_achievers_[id].clear();
  first_achievers_[id].clear();

  landmark_to_id_.erase(id_to_landmark_[id]);
  id_to_landmark_[id].Clear();
}

void LandmarkGraph::Delete(const Landmark &landmark) {
  if (!IsIn(landmark)) return;
  size_t id = landmark_to_id_[landmark];
  Delete(id);
}

void LandmarkGraph::GetAncestors(size_t start, vector<size_t> &ancestors) {
  ancestors.clear();
  closed_.clear();
  open_ = stack<size_t>();

  open_.push(start);
  closed_.insert(start);
  while (!open_.empty()) {
    size_t current = open_.top();
    open_.pop();
    for (auto parent : GetInitIdsByTermId(current)) {
      if (closed_.find(parent) != closed_.end()) continue;
      closed_.insert(parent);
      ancestors.push_back(parent);
      open_.push(parent);
    }
  }
}

void LandmarkGraph::RemoveEdge(size_t start) {
  size_t parent = path_.back();
  auto start_it = find(path_.begin(), path_.end(), start);
  for (auto it = start_it; it != path_.end(); ++it) {
    size_t child = *it;
    if (GetOrderingType(parent, child) == OBEDIENT) {
      DeleteOrdering(parent, child);
      return;
    }
    parent = child;
  }

  parent = path_.back();
  for (auto it = start_it; it != path_.end(); ++it) {
    size_t child = *it;
    if (GetOrderingType(parent, child) == REASONABLE) {
      DeleteOrdering(parent, child);
      return;
    }
    parent = child;
  }
}

bool LandmarkGraph::FindCycle(size_t current) {
  if (closed_.find(current) != closed_.end()) {
    RemoveEdge(current);
    return true;
  }
  path_.push_back(current);
  closed_.insert(current);
  for (auto child : GetTermIdsByInitId(current))
    if (FindCycle(child)) return true;
  path_.pop_back();
  closed_.erase(current);
  return false;
}

int LandmarkGraph::RemoveCycles(size_t start) {
  int cycles = -1;
  do {
    path_.clear();
    closed_.clear();
    ++cycles;
  } while (FindCycle(start));
  return cycles;
}

bool TypeCheck(const vector<int> &allowed_types, int type) {
  auto result = find(allowed_types.begin(), allowed_types.end(), type);
  return result != allowed_types.end();
}

} //namespace rwls
