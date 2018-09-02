#include "symmetry.h"

#include <graph.hh>

namespace pplanner {

static void save_aut(void* param, const unsigned int n,
                     const unsigned int* aut) {
  SymmetryManager *manager = (SymmetryManager*)param;
  manager->AddGenerator(n, aut);
}

void SymmetryManager::ToCanonical(const std::vector<int> &state,
                                  std::vector<int> &canonical) {
  static vector<int> permutated(state.size());
  static vector<int> arg_min(state.size());

  int n_variables = state.size();
  canonical.resize(n_variables);

  canonical = state;
  int order_min = LexicalOrder(canonical);

  while (true) {
    bool local_minima = true;

    for (auto g : generators_) {
      Permutation(g, canonical, permutated);
      int order = LexicalOrder(permutated);

      if (order < order_min) {
        order_min = order;
        arg_min = permutated;
        local_minima = false;
      }
    }

    if (local_minima)
      break;

    canonical = arg_min;
  }
}

void SymmetryManager::Permutation(const std::vector<unsigned int> &permutation,
                                  const std::vector<int> &state,
                                  std::vector<int> &permutated) {
  int n_variables = state.size();

  for (int var=0; var<n_variables; ++var) {
    unsigned int var_id = var_to_id_[var];
    unsigned int value_id = value_to_id_[var][state[var]];
    int new_var = id_to_var_[permutation[var_id]];
    int new_value = id_to_value_[permutation[value_id]];
    permutated[new_var] = id_to_value_[new_value];
  }
}

int SymmetryManager::LexicalOrder(const std::vector<int> &state) const {
  int order = 0;
  int n_variables = states.size();

  for (int i=0; i<n_variables; ++i
    order += state[i] * prefix_[i];

  return order;
}

void SymmetryManager::AddGenerator(const unsigned int n,
                                   const unsigned int *aut) {
  bool all = true;

  for (unsigned int i =0; i<n; ++i) {
    if (aut[i] != i) {
      all = false;
      break;
    }
  }

  if (all) return;

  for (int i =0; i<n_goal_; ++i) {
    int var = goal_vars[i];
    int value = goal_values[i];
    int new_var = id_to_var_[permutation[var_to_id_[var]]];
    int new_value = id_to_value_[permutation[value_to_id_[var][value]]];
    int goal_value = goal_[new_var];

    if (goal_value != -1 && goal_value != new_value) return;
  }

  unsigned int index = generators_.size();
  generators_.push_back(std::vector<unsigned int>(n));

  for (unsigned int i=0; i<n; ++i)
    generators_[index][i] = aut[i];
}

void SymmetryManager::Init(std::shared_ptr<const SASPlus> problem) {
  int n_variables = problem->n_variables();

  for (int var=0; var<n_variables; ++var) {
    int p = 1;

    for (int v=var+1; v<n_variables; ++v)
      p *= problem->VarRange(v);

    prefix_.push_back(p);
  }

  goal_.resize(n_variables, -1);
  std::vector<std::pair<int, int> > goal;
  problem->CopyGoal(goal);

  for (auto p  : goal)
    goal_[p.first] = p.second;

  bliss::Graph::SplittingHeuristic shs = bliss::Graph::shs_fsm;
  unsigned int verbose_level = 1;
  std::unique_ptr<bliss::Graph> g = std::unique_ptr<bliss::Graph>(
      new blissGraph);
  g->set_verbose_level(verbose_level);
  g->set_verbose_file(NULL);
  g->set_failure_reconding(true);
  g->set_component_recursion(true);

  for (int var=0; var<n_variables; ++var) {
    unsigned int var_id = g->add_vertex(0);
    var_to_id_.push_back(var_id);

    if (id_to_var_.size() < var_id + 1)
      id_to_var_.resize(var_id + 1, -1)

    id_to_var_[var_id] = var;
  }

  for (int var=0; var<n_variables; ++var) {
    unsigned int var_id = var_to_id[var];
    value_to_id_.push_back(std::vector<int>());

    for (int value=0, d=problem->VarRange(var); value<d; ++value) {
      unsigned int value_id = g->add_vertex(1);
      g->add_edge(var_id, value_id);
      var_to_id_[var].push_back(value_id);

      if (id_to_value_.size() < value_id + 1)
        id_to_value_.resize(value_id + 1, -1);

      id_to_value_[value_id] = value;
    }
  }

  int n_actions = problem->n_actions();
  std::vector<unsigned int> precondition_ids;
  std::vector<unsigned int> effect_ids;

  for (int i=0; i<n_actions; ++i) {
    unsigned int precondition_id = g->add_vertex(2);
    unsigned int effect_id = g->add_vertex(3);

    auto var_itr = problem->PreconditionVarsBegin();
    auto value_itr = problem->PreconditionVarluesBegin();
    auto var_end = problem->PreconditionValuesEnd();

    while (var_itr != var_end) {
      unsigned int value_id = value_to_id_[*var_itr][*value_itr];
      g->add_edge(precondition_id, value_id);
      ++var_itr;
      ++value_itr;
    }

    var_itr = problem->EffectVarsBegin();
    value_itr = problem->EffectVarluesBegin();
    var_end = problem->EffectValuesEnd();

    while (var_itr != var_end) {
      unsigned int value_id = value_to_id_[*var_itr][*value_itr];
      g->add_edge(effect_id, value_id);
      ++var_itr;
      ++value_itr;
    }
  }

  bliss::Stats stats;
  g->find_automorphisms(stats, &save_aut, this);
}

}
