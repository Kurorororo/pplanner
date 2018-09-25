#include "symmetry.h"

#include <graph.hh>
#include <iostream>

namespace pplanner {

using std::vector;

static void save_aut(void* param, const unsigned int n,
                     const unsigned int* aut) {
  SymmetryManager *manager = (SymmetryManager*)param;
  manager->AddGenerator(n, aut);
}

void SymmetryManager::ToCanonical(const vector<int> &state,
                                  vector<int> &canonical) const {
  static vector<int> permutated(state.size());
  static vector<int> arg_min(state.size());

  canonical = state;

  while (true) {

    bool local_minima = true;

    for (int i=0, n=var_permutations_.size(); i<n; ++i) {
      Permutate(i, canonical, permutated);

      if (permutated < canonical) {
        arg_min = permutated;
        local_minima = false;
      }
    }

    if (local_minima)
      break;

    canonical = arg_min;
  }
}

void SymmetryManager::ToCanonical(const vector<int> &state,
                                  vector<int> &canonical,
                                  vector<int> &generators) const {
  static vector<int> permutated(state.size());
  static vector<int> arg_min(state.size());

  generators.clear();
  canonical = state;

  while (true) {

    bool local_minima = true;
    int sigma = -1;

    for (int i=0, n=var_permutations_.size(); i<n; ++i) {
      Permutate(i, canonical, permutated);

      if (permutated < canonical) {
        arg_min = permutated;
        local_minima = false;
        sigma = i;
      }
    }

    if (local_minima)
      break;

    canonical = arg_min;
    generators.push_back(sigma);
  }
}

void SymmetryManager::Permutate(int i, const vector<int> &state,
                                vector<int> &permutated) const {
  int n_variables = state.size();

  for (int var=0; var<n_variables; ++var) {
    int new_var = var_permutations_[i][var];
    int new_value = value_permutations_[i][var][state[var]];
    permutated[new_var] = new_value;
  }
}

void SymmetryManager::InversePermutate(int i, const vector<int> &state,
                                       vector<int> &permutated) const {
  int n_variables = state.size();

  for (int var=0; var<n_variables; ++var) {
    int new_var = inverse_var_permutations_[i][var];
    int new_value = inverse_value_permutations_[i][var][state[var]];
    permutated[new_var] = new_value;
  }
}

void SymmetryManager::AddGenerator(const unsigned int n,
                                   const unsigned int *aut) {
  int n_variables = var_to_id_.size();
  std::vector<int> var_permutation(n_variables);
  std::vector<std::vector<int> > value_permutation(n_variables);

  bool all = true;

  for (int var=0; var<n_variables; ++var) {
    int new_var = id_to_var_[aut[var_to_id_[var]]];
    var_permutation[var] = new_var;
    value_permutation.resize(value_permutation.size() + 1);

    if (all && var != new_var) all = false;

    for (int value=0; value<problem_->VarRange(var); ++value) {
      int new_value = id_to_value_[aut[value_to_id_[var][value]]];
      value_permutation[var].push_back(new_value);

      if (all && value != new_value) all = false;
    }
  }

  if (all) return;

  auto permutated = goal_;

  for (int var =0; var<n_variables; ++var) {
    int value = goal_[var];
    int new_var = var_permutation[var];
    int new_value = value == -1 ? value : value_permutation[var][value];
    int goal_value = goal_[new_var];

    if (goal_value != new_value) return;
  }

  var_permutations_.push_back(var_permutation);
  value_permutations_.push_back(value_permutation);
}

void SymmetryManager::AddInverseGenerators() {
  inverse_value_permutations_.resize(value_permutations_.size(),
                                     vector<vector<int> >());

  for (int i=0, n=var_permutations_.size(); i<n; ++i) {
    vector<int> var_permutation(var_permutations_[i].size());

    for (int j=0, m=var_permutations_[i].size(); j<m; ++j) {
      var_permutation[var_permutations_[i][j]] = j;
      vector<int> value_permutation(value_permutations_[i][j].size());

      for (int k=0, l=value_permutations_[i][j].size(); k<l; ++k)
        value_permutation[value_permutations_[i][j][k]] = k;

      inverse_value_permutations_[i].push_back(value_permutation);
    }

    inverse_var_permutations_.push_back(var_permutation);
  }
}

void SymmetryManager::Init(std::shared_ptr<const SASPlus> problem) {
  int n_variables = problem->n_variables();

  goal_.resize(n_variables, -1);
  vector<std::pair<int, int> > goal;
  problem->CopyGoal(goal);

  for (auto p  : goal)
    goal_[p.first] = p.second;

  bliss::Graph::SplittingHeuristic shs = bliss::Graph::shs_fsm;
  unsigned int verbose_level = 1;
  std::unique_ptr<bliss::Graph> g = std::unique_ptr<bliss::Graph>(
      new bliss::Graph);
  g->set_splitting_heuristic(shs);
  g->set_verbose_level(verbose_level);
  g->set_verbose_file(NULL);
  g->set_failure_recording(true);
  g->set_component_recursion(true);

  for (int var=0; var<n_variables; ++var) {
    unsigned int var_id = g->add_vertex(0);
    var_to_id_.push_back(var_id);

    if (id_to_var_.size() < var_id + 1)
      id_to_var_.resize(var_id + 1, -1);

    id_to_var_[var_id] = var;
  }

  for (int var=0; var<n_variables; ++var) {
    unsigned int var_id = var_to_id_[var];
    value_to_id_.resize(value_to_id_.size() + 1);

    for (int value=0, d=problem->VarRange(var); value<d; ++value) {
      unsigned int value_id = g->add_vertex(1);
      g->add_edge(var_id, value_id);
      value_to_id_[var].push_back(value_id);

      if (id_to_value_.size() < value_id + 1)
        id_to_value_.resize(value_id + 1, -1);

      id_to_value_[value_id] = value;
    }
  }

  int n_actions = problem->n_actions();

  for (int i=0; i<n_actions; ++i) {
    unsigned int precondition_id = g->add_vertex(2);
    unsigned int effect_id = g->add_vertex(3 + problem->ActionCost(i));
    g->add_edge(precondition_id, effect_id);

    auto var_itr = problem->PreconditionVarsBegin(i);
    auto value_itr = problem->PreconditionValuesBegin(i);
    auto var_end = problem->PreconditionVarsEnd(i);

    while (var_itr != var_end) {
      unsigned int value_id = value_to_id_[*var_itr][*value_itr];
      g->add_edge(precondition_id, value_id);
      ++var_itr;
      ++value_itr;
    }

    var_itr = problem->EffectVarsBegin(i);
    value_itr = problem->EffectValuesBegin(i);
    var_end = problem->EffectVarsEnd(i);

    while (var_itr != var_end) {
      unsigned int value_id = value_to_id_[*var_itr][*value_itr];
      g->add_edge(effect_id, value_id);
      ++var_itr;
      ++value_itr;
    }
  }

  bliss::Stats stats;
  g->find_automorphisms(stats, &save_aut, this);
  AddInverseGenerators();
}

void SymmetryManager::Dump() const {
  std::cout << "#permutations=" << var_permutations_.size() << std::endl;

  for (int i=0, n=var_permutations_.size(); i<n; ++i) {
    std::cout << "permutation " << i << std::endl;

    for (int var=0, m=var_permutations_[i].size(); var<m; ++var) {
      int new_var = var_permutations_[i][var];
      std::cout << "var" << var << "->var" << new_var << std::endl;

      for (int value=0, l=problem_->VarRange(var); value<l; ++value) {
        int new_value = value_permutations_[i][var][value];
        std::cout << value << "->" << new_value << ",";
      }

      std::cout << std::endl;
    }

    std::cout << std::endl;
  }
}

} // namespace pplanner
