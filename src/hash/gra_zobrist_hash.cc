#include "hash/gra_zobrist_hash.h"

#include <random>

#include "dtg.h"
#include "utils/arg_sort.h"

#include <iostream>

namespace pplanner {

using std::vector;

std::vector<double> CalculateFluency(std::shared_ptr<const SASPlus> problem) {
  std::vector<int> n_operators(problem->n_variables(), 0);

  for (int i = 0, n = problem->n_actions(); i < n; ++i) {
    auto itr = problem->EffectVarsBegin(i);
    auto end = problem->EffectVarsEnd(i);

    for (; itr != end; ++itr)
      ++n_operators[*itr];

    if (problem->HasConditionalEffects(i))
      for (int j = 0, m = problem->NConditionalEffects(i); j < m; ++j)
        ++n_operators[problem->ConditionalEffectVar(i, j)];
  }

  std::vector<double> fluencies(problem->n_variables());
  double n = static_cast<double>(problem->n_variables());

  for (int i = 0; i < problem->n_variables(); ++i)
    fluencies[i] = static_cast<double>(n_operators[i]) / n;

  return fluencies;
}

GRAZobristHash::GRAZobristHash(std::shared_ptr<const SASPlus> problem,
                               uint32_t seed, bool greedy, double threshold)
  : problem_(problem),
    cuts_(problem->n_variables()),
    ignore_(problem->n_variables(), false),
    array_(problem->n_variables()) {
  auto dtgs = InitializeDTGs(problem);

  if (threshold > 0.0) {
    auto fluencies = CalculateFluency(problem);
    int n_top = static_cast<double>(problem->n_variables()) * threshold;
    auto indices = ArgSort(fluencies, true);

    for (int i = 0; i < n_top; ++i) {
      ignore_[indices[i]] = true;
      std::cout << "ignore " << indices[i] << " f=" << fluencies[indices[i]] << std::endl;
    }
  }

  std::vector<int> cut;

  for (int i = 0; i < problem->n_variables(); ++i) {
    if (ignore_[i]) continue;

    if (greedy)
      dtgs[i]->GreedyCut(cut);
    else
      dtgs[i]->SparsestCut(cut);

    cuts_[i] = cut;
  }

  std::mt19937 mt(seed);

  for (auto &a : array_)
    for (auto &v : a)
      v = mt();
}

uint32_t GRAZobristHash::operator()(const vector<int> &state) const {
  uint32_t seed = 0;

  for (int i=0, n=state.size(); i<n; ++i) {
    if (ignore_[i]) continue;
    seed ^= array_[i][cuts_[i][state[i]]];
  }

  return seed;
}

uint32_t GRAZobristHash::HashByDifference(int action, uint32_t seed,
                                          const vector<int> &parent,
                                          const vector<int> &state) {
  auto itr = problem_->EffectVarsBegin(action);
  auto end = problem_->EffectVarsEnd(action);

  for (; itr != end; ++itr) {
    int var = *itr;
    if (ignore_[var]) continue;
    seed = seed ^ array_[var][cuts_[var][parent[var]]]
                ^ array_[var][cuts_[var][state[var]]];
  }

  if (problem_->use_conditional() && problem_->HasConditionalEffects(action)) {
    int n = problem_->NConditionalEffects(action);

    for (int j = 0; j < n; ++j) {
      int var = problem_->ConditionalEffectVar(action, j);
      if (ignore_[var]) continue;
      seed = seed ^ array_[var][cuts_[var][parent[var]]]
                  ^ array_[var][cuts_[var][state[var]]];
    }
  }
  return seed;
}

} // namespace pplanner
