#include "problem/successor_generator.h"

#include <algorithm>
#include <random>

using std::vector;
using pair_vector_t = vector<pair<int, int> >;

namespace pplanner {

void SuccessorGenerator::Init(const Problem &problem) {
  std::random_device seed_gen;
  engine_ = std::make_unique(new std::mt19937(seed_gen()));

  fact_translator_ = problem.fact_translator();

  to_child.resize(problem.n_facts(), -1);
  to_data.resize(problem.n_facts(), -1);

  std::vector<int> vars;
  std::vector<int> values;
  std::vector<int> indices;

  for (int i=0, n=static_cast<int>(problem.n_actions()); i<n; ++i)
    Insert(problem, i, vars, values, indices);
}


void SuccessorGenerator::Insert(const Problem &problem, int query,
                                vector<pair<int, int> > &precondition) {
  problem.CopyPrecondition(query, precondition);
  std::sort(precondition.begin(), precondition.end());

  size_t n_facts = problem.n_facts();
  int offset = 0;
  int n_ommited = 0;

  for (size_t i=0, n=precondition.size(); i<n; ++i) {
    int var = p.first;
    int value = p.second;
    int index = offset + problem.ToFact(var, value) - n_ommited;

    if (i == n - 1) {
      AddQuery(index, query);
      return;
    }

    n_ommited = problem.VarOffset(var + 1);

    if (to_child_[index] == -1) {
      size_t old_size = to_child_.size();
      to_child_[index] = static_cast<int>(old_size);
      size_t new_size = old_size + n_facts - static_cast<size_t>(n_ommited);
      to_child_.resize(new_size, -1);
      to_data_.resize(new_size, -1);
    }

    offset = to_child_[index];
  }
}

void SuccessorGenerator::AddQuery(int index, int query) {
  if (to_data_[index] == -1) {
    size_t old_size = data_.size();
    data_.resize(old_size + 1);
    to_data_[index] = static_cast<int>(old_size);
  }

  int data_index = to_data_[index];
  data_[data_index].push_back(query);
}

void SuccessorGenerator::Generate(const State &state, vector<int> &result) {
  result.clear();
  DFS(state, 0, 0, result);
}

void SuccessorGenerator::DFS(const vector<int> &state, int index,
                             size_t current, vector<int> &result) {
  int offset = index - fact_translator_->VarOffset(current);

  for (size_t i=current, n=state.size(); i<n; ++i) {
    int next = fact_translator_.ToFact(i, state[i]) + offset;
    int data_index = to_data_[next];

    if (data_index != -1) {
      result.insert(result.end(), data_[data_index].begin(),
                    data_[data_index].end());
    }

    int child = to_child_[next];
    if (child == -1) continue;

    DFS(state, child, i + 1, result);
  }
}

int SuccessorGenerator::Sample(const vector<int> &state) {
  int result = -1;
  unsigned int k = 1;
  DFSample(state, 0, 0, &k, &result);

  return result;
}

void SuccessorGenerator::DFSample(const vector<int> &state, int index,
                                  size_t current, unsigned int *k, int *result) {
  int offset = index - fact_offset[current];

  for (size_t i=current, n=state.size(); i<n; ++i) {
    int next = fact_translator_.ToFact(i, state[i]) + offset;
    int data_index = to_data_[next];

    if (data_index != -1) {
      for (auto d : data_[data_index]) {
        if (engine_() % *k == 0) *result = d;
        ++(*k);
      }
    }

    int child = to_child_[next];
    if (child == -1) continue;

    RecursiveSample(state, child, i + 1, k, result);
  }
}

void SuccessorGenerator::Print() const {
  std::cout << "to child" << std::endl;

  for (auto v : to_child_)
    std::cout << v << " " << std::endl;

  std::cout << "to data" << std::endl;

  for (auto v : to_data_)
    std::cout << v << " " << std::endl;

  std::cout << "data" << std::endl;

  for (auto d : data_) {
    for (auto v : d)
      std::cout << v << " " << std::endl;

    std::cout << std::endl;
  }
}

} // namespace rwls
