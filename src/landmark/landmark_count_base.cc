#include "landmark/landmark_count_base.h"

#include "utils/bit_vector.h"

namespace pplanner {

using std::unordered_set;
using std::vector;

bool LandmarkCountBase::IsLeaf(int lm_id, const uint8_t *accepted) const {
  for (auto parent_id : graph_->GetInitIdsByTermId(lm_id))
    if (!Get(accepted, parent_id)) return false;

  return true;
}

int LandmarkCountBase::ReachedSize(const vector<int> &state,
                                   const uint8_t *parent_accepted,
                                   uint8_t *accepted) {
  int size = 0;

  for (int psi_id = 0, n = graph_->landmark_id_max(); psi_id < n; ++psi_id) {
    if (Get(parent_accepted, psi_id)) {
      Up(accepted, psi_id);
      ++size;
      continue;
    }

    const Landmark &psi = graph_->GetLandmark(psi_id);
    if (psi.IsEmpty() || !psi.IsImplicated(state)) continue;

    if (IsLeaf(psi_id, parent_accepted)) {
      Up(accepted, psi_id);
      ++size;
    }
  }

  return size;
}

int LandmarkCountBase::NeededSize(const vector<int> &state,
                                  const uint8_t *accepted) {
  int size = 0;

  for (int phi_id = 0, n = graph_->landmark_id_max(); phi_id < n; ++phi_id) {
    if (!Get(accepted, phi_id)) continue;
    status_[phi_id] = REACHED;

    const Landmark &phi = graph_->GetLandmark(phi_id);
    if (phi.IsImplicated(state)) continue;

    if (graph_->IsGoal(phi_id)) {
      status_[phi_id] = NEEDED;
      ++size;
      continue;
    }

    for (auto psi_id : graph_->GetTermIdsByInitId(phi_id)) {
      auto ordering_type = graph_->GetOrderingType(phi_id, psi_id);

      if (ordering_type == LandmarkGraph::GREEDY && !Get(accepted, psi_id)) {
        status_[phi_id] = NEEDED;
        ++size;
        break;
      }
    }
  }

  return size;
}

int LandmarkCountBase::Evaluate(const vector<int> &state,
                                const uint8_t *parent_accepted,
                                uint8_t *accepted) {
  if (problem_->IsGoal(state)) return 0;

  std::fill(status_.begin(), status_.end(), NOT_REACHED);
  reached_size_ = 0;

  if (parent_accepted == nullptr) {
    for (int i = 0, n = graph_->landmark_id_max(); i < n; ++i) {
      const Landmark &psi = graph_->GetLandmark(i);

      if (!psi.IsEmpty() && psi.IsImplicated(state) &&
          graph_->GetInitIdsByTermId(i).empty()) {
        status_[i] = REACHED;
        Up(accepted, i);
        ++reached_size_;
      }
    }
  } else {
    reached_size_ = ReachedSize(state, parent_accepted, accepted);
  }

  int needed_size = NeededSize(state, accepted);

  for (int i = 0, n = graph_->landmark_id_max(); i < n; ++i) {
    const Landmark &psi = graph_->GetLandmark(i);

    if (!psi.IsEmpty() &&
        ((status_[i] == NOT_REACHED && graph_->GetFirstAchieversSize(i) == 0) ||
         (status_[i] == NEEDED && graph_->GetPossibleAchieversSize(i) == 0)))
      return -1;
  }

  return graph_->n_landmarks() - reached_size_ + needed_size;
}

int LandmarkCountBase::Evaluate(const vector<int> &state,
                                const vector<int> &applicable,
                                const uint8_t *parent_accepted,
                                uint8_t *accepted,
                                unordered_set<int> &preferred,
                                bool next_step_only) {
  static std::vector<int> facts;
  static std::vector<int> disjunctive_goals;

  int h = Evaluate(state, parent_accepted, accepted);

  if (h == 0 || h == -1) return h;

  preferred.clear();
  NextStepOperators(state, applicable, accepted, preferred);

  if (next_step_only) return h;

  if (preferred.empty()) {
    disjunctive_goals.clear();

    for (int i = 0, n = graph_->landmark_id_max(); i < n; ++i) {
      if (!Get(accepted, i) && IsLeaf(i, accepted)) {
        auto l = graph_->GetLandmark(i);

        for (int j = 0, m = l.size(); j < m; ++j) {
          int f = problem_->Fact(l.VarValue(j));
          disjunctive_goals.push_back(f);
        }
      }
    }

    StateToFactVector(problem_, state, facts);

    if (disjunctive_goals.empty())
      rpg_->PlanCost(facts, preferred);
    else
      rpg_->DisjunctiveHelpful(facts, disjunctive_goals, preferred);

    if (preferred.empty()) return -1;
  }

  return h;
}

bool LandmarkCountBase::IsInteresting(int lm_id, const vector<int> &state,
                                      const uint8_t *accepted) const {
  const Landmark &lm = graph_->GetLandmark(lm_id);

  if (reached_size_ == static_cast<int>(graph_->n_landmarks()))
    return graph_->IsGoal(lm_id) && !lm.IsImplicated(state);

  return !Get(accepted, lm_id) && IsLeaf(lm_id, accepted);
}

void LandmarkCountBase::NextStepOperators(const vector<int> &state,
                                          const vector<int> &applicable,
                                          const uint8_t *accepted,
                                          unordered_set<int> &preferred) {
  static vector<int> fact_lm_operators;
  static vector<int> disj_lm_operators;

  fact_lm_operators.clear();
  disj_lm_operators.clear();

  for (auto a : applicable) {
    auto var_iter = problem_->EffectVarsBegin(a);
    auto var_end = problem_->EffectVarsEnd(a);
    auto values_iter = problem_->EffectValuesBegin(a);

    for (; var_iter != var_end; ++var_iter) {
      int f = problem_->Fact(*var_iter, *values_iter);
      ++values_iter;
      int lm_id = graph_->FactToId(f);

      if (lm_id != -1 && IsInteresting(lm_id, state, accepted)) {
        if (graph_->GetLandmark(lm_id).IsFact())
          fact_lm_operators.push_back(a);
        else
          disj_lm_operators.push_back(a);
      }
    }
  }

  if (fact_lm_operators.empty()) {
    for (auto o : disj_lm_operators) preferred.insert(o);
  } else {
    for (auto o : fact_lm_operators) preferred.insert(o);
  }
}

}  // namespace pplanner
