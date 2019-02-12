#include "multithread_search/abstract_graph.h"

#include <algorithm>
#include <numeric>
#include <iostream>

#include "dtg.h"
#include "utils/lexical_order.h"

namespace pplanner {

AbstractGraph::AbstractGraph(std::shared_ptr<const SASPlus> problem,
                             const std::vector<int> &vars)
  : max_out_degree_(0), vars_(vars), ranges_(vars.size()) {

  for (int i = 0, n = ranges_.size(); i < n; ++i)
    ranges_[i] = problem->VarRange(vars[i]);

  int n_nodes = 1;

  for (int i = 0, n = ranges_.size(); i < n; ++i)
    n_nodes *= ranges_[i];

  n_nodes_ = n_nodes;
  matrix_.resize(n_nodes, std::vector<bool>(n_nodes, false));
  to_parents_.resize(n_nodes);
  to_children_.resize(n_nodes);
  inference_scope_.resize(n_nodes);
  ConstructGraph(problem);
}

void AbstractGraph::BuildInferenceScope() {
  for (int i = 0, n = to_children_.size(); i < n; ++i) {
    inference_scope_[i] = to_children_[i];

    for (int j : to_children_[i]) {
      for (auto k : to_parents_[j]) {
        if (i == k) continue;
        auto result = std::find(inference_scope_[i].begin(),
                                inference_scope_[i].end(), k);

        if (result == inference_scope_[i].end())
          inference_scope_[i].push_back(k);
      }
    }
  }
}

int AbstractGraph::StateToNode(const std::vector<int> &state,
                               std::vector<int> &values) const {
  for (int i = 0, n = vars.size(); i < n; ++i)
    values[i] = state[vars_[i]];

  return LexicalOrder(values, ranges_);
}

void AbstractGraph::Dump() const {
  std::cout << "children" << std::endl;

  for (int i = 0; i < n_nodes_; ++i) {
    auto state = OrderToValues(i, ranges_);

    std::cout << i << "(";

    for (auto v : state)
      std::cout << v << " ";

    std::cout << ") -> ";

    for (auto c : to_children_[i])
      std::cout << c << " ";

    std::cout << std::endl;
  }

  std::cout << "inferece scope" << std::endl;

  for (int i = 0; i < n_nodes_; ++i) {
    auto state = OrderToValues(i, ranges_);

    std::cout << i << "(";

    for (auto v : state)
      std::cout << v << " ";

    std::cout << ") -> ";

    for (auto c : inference_scope_[i])
      std::cout << c << " ";

    std::cout << std::endl;
  }

  std::cout << "vars:";

  for (int i = 0, n = vars_.size(); i < n; ++i)
    std::cout << " " << vars_[i];

  std::cout << std::endl;
  std::cout << "#nodes=" << n_nodes_ << std::endl;
  std::cout << "#successoors=" << max_out_degree_ << std::endl;
  std::cout << "delta=" << Delta() << std::endl;
}

bool AbstractGraph::IsActionRelevant(std::shared_ptr<const SASPlus> problem,
                                     int action) const {
  auto iter = problem->EffectVarsBegin(action);
  auto end = problem->EffectVarsEnd(action);

  for (; iter != end; ++iter)
    if (std::find(vars_.begin(), vars_.end(), *iter) != vars_.end())
      return true;

  if (problem->HasConditionalEffects(action)) {
    for (int i = 0; i < problem->NConditionalEffects(action); ++i) {
      int var = problem->ConditionalEffectVar(action, i);

      if (std::find(vars_.begin(), vars_.end(), var) != vars_.end())
        return true;
    }
  }

  return false;
}

void AbstractGraph::ApplyEffect(std::shared_ptr<const SASPlus> problem,
                                int action, const std::vector<int> &state,
                                std::vector<int> &child) const {
  child = state;
  auto iter = problem->EffectVarsBegin(action);
  auto end = problem->EffectVarsEnd(action);
  auto value_iter = problem->EffectValuesBegin(action);

  while (iter != end) {
    for (int i = 0, n = vars_.size(); i < n; ++i)
      if (vars_[i] == *iter)
        child[i] = *value_iter;

    ++iter;
    ++value_iter;
  }

  if (problem->HasConditionalEffects(action)) {
    for (int i = 0; i < problem->NConditionalEffects(action); ++i) {
      int var = problem->ConditionalEffectVar(action, i);

      if (std::find(vars_.begin(), vars_.end(), var) == vars_.end())
        continue;

      bool condition = true;

      for (int j = 0; j < problem->NEffectCondition(action, i); ++j) {
        int c_var = problem->EffectConditionVar(action, i, j);

        for (int k = 0, m = vars_.size(); k < m; ++k) {
          if (vars_[k] != c_var) continue;

          if (state[k] != problem->EffectConditionValue(action, i, j)) {
            condition = false;
            break;
          }
        }

        if (!condition) break;
      }

      if (condition)
        child[var] = problem->ConditionalEffectValue(action, i);
    }
  }
}

void AbstractGraph::AddEdge(
    std::shared_ptr<const SASPlus> problem,
    int action,
    const std::vector<std::vector<int> > &possible_values,
    int index,
    std::vector<int> &state) {
  if (index == vars_.size()) {
    auto child = state;
    ApplyEffect(problem, action, state, child);
    int p = LexicalOrder(state, ranges_);
    int c = LexicalOrder(child, ranges_);

    if (p != c && !matrix_[p][c]) {
      matrix_[p][c] = true;
      to_parents_[c].push_back(p);
      to_children_[p].push_back(c);

      if (to_children_[p].size() > max_out_degree_)
        max_out_degree_ = to_children_[p].size();
    }

    return;
  }

  for (auto value : possible_values[index]) {
    auto s = state;
    s[index] = value;
    AddEdge(problem, action, possible_values, index + 1, s);
  }
}

void AbstractGraph::ConstructGraph(std::shared_ptr<const SASPlus> problem) {
  std::vector<std::vector<int> > possible_values(vars_.size());
  std::vector<int> state(vars_.size());

  for (int i = 0, n = problem->n_actions(); i < n; ++i) {
    if (!IsActionRelevant(problem, i)) continue;

    for (auto &v : possible_values)
      v.clear();

    auto var_iter = problem->PreconditionVarsBegin(i);
    auto var_end = problem->PreconditionVarsEnd(i);
    auto value_iter = problem->PreconditionValuesBegin(i);

    while (var_iter != var_end) {
      for (int j = 0, m = vars_.size(); j < m; ++j)
        if (vars_[j] == *var_iter)
          possible_values[j].push_back(*value_iter);

      ++var_iter;
      ++value_iter;
    }

    for (int j = 0, m = vars_.size(); j < m; ++j) {
      if (possible_values[j].empty()) {
        possible_values[j].resize(ranges_[j]);
        std::iota(possible_values[j].begin(), possible_values[j].end(), 0);
      }
    }

    AddEdge(problem, i, possible_values, 0, state);
  }
}

std::shared_ptr<AbstractGraph> GreedySelect(
    std::shared_ptr<const SASPlus> problem,
    const std::vector<int> &candidates,
    const std::vector<int> &selected,
    int max_n_nodes) {
  int base_size = 1;

  for (auto v : selected)
    base_size *= problem->VarRange(v);

  std::shared_ptr<AbstractGraph> best_graph = nullptr;
  std::vector<int> tmp_vars;

  for (auto v : candidates) {
    if (!selected.empty()
        && std::find(selected.begin(), selected.end(), v) != selected.end())
      continue;

    int range = problem->VarRange(v);

    if (range < 3 || base_size * range > max_n_nodes)
      continue;

    tmp_vars = selected;
    tmp_vars.push_back(v);

    auto graph = std::make_shared<AbstractGraph>(problem, tmp_vars);

    if (best_graph == nullptr || graph->Delta() < best_graph->Delta())
      best_graph = graph;
  }

  if (best_graph == nullptr) {
    for (auto v : candidates) {
      if (problem->VarRange(v) != 2 || base_size * 2 > max_n_nodes)
        continue;

      if (!selected.empty()
          && std::find(selected.begin(), selected.end(), v) != selected.end())
        continue;

      tmp_vars = selected;
      tmp_vars.push_back(v);
      auto graph = std::make_shared<AbstractGraph>(problem, tmp_vars);

      if (best_graph == nullptr || graph->Delta() < best_graph->Delta())
        best_graph = graph;
    }
  }

  return best_graph;
}

std::shared_ptr<AbstractGraph> ConstructByDTG(
    std::shared_ptr<const SASPlus> problem,
    int max_n_nodes) {
  auto dtgs = InitializeDTGs(problem);
  std::vector<int> vars;
  std::vector<bool> done(dtgs.size(), false);
  int n_nodes = 1;

  while (true) {
    int min_degree = -1;
    int arg_min = -1;

    for (int i = 0, n = dtgs.size(); i < n; ++i) {
      int range = problem->VarRange(i);

      if (done[i] || (range < 3 || n_nodes * range > max_n_nodes))
        continue;

      int degree = dtgs[i]->OutDegreeMax();

      if (min_degree == -1 || degree < min_degree) {
        min_degree = degree;
        arg_min = i;
      }
    }

    if (arg_min == -1) {
      for (int i = 0, n = dtgs.size(); i < n; ++i) {
        int range = problem->VarRange(i);

        if (done[i] || (range != 2 || n_nodes * range > max_n_nodes))
          continue;

        int degree = dtgs[i]->OutDegreeMax();

        if (min_degree == -1 || degree < min_degree) {
          min_degree = degree;
          arg_min = i;
        }
      }
    }

    if (arg_min == -1) break;

    vars.push_back(arg_min);
    done[arg_min] = true;
    n_nodes *= problem->VarRange(arg_min);
  }

  return std::make_shared<AbstractGraph>(problem, vars);
}

} // namespace pplanner
