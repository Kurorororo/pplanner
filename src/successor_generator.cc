#include "successor_generator.h"

#include <algorithm>
#include <iostream>
#include <random>

using std::pair;
using std::vector;

namespace pplanner {

void SuccessorGenerator::Init(const SASPlus &problem) {
  std::random_device seed_gen;
  engine_ = std::mt19937(seed_gen());

  facts_ = problem.facts();

  to_child_.resize(problem.n_facts(), -1);
  to_data_.resize(problem.n_facts(), -1);

  vector<pair<int, int> > precondition;

  for (int i=0, n=static_cast<int>(problem.n_actions()); i<n; ++i)
    Insert(problem, i, precondition);
}


void SuccessorGenerator::Insert(const SASPlus &problem, int query,
                                vector<pair<int, int> > &precondition) {
  assert(nullptr != facts_);

  problem.CopyPrecondition(query, precondition);
  std::sort(precondition.begin(), precondition.end());

  size_t n_facts = facts_->size();
  int offset = 0;
  int n_ommited = 0;

  for (size_t i=0, n=precondition.size(); i<n; ++i) {
    int var = precondition[i].first;
    int value = precondition[i].second;
    int index = offset + facts_->Fact(var, value) - n_ommited;

    if (i == n - 1) {
      AddQuery(index, query);
      return;
    }

    n_ommited = facts_->VarBegin(var + 1);

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

void SuccessorGenerator::DFS(const vector<int> &state, int index,
                             size_t current, vector<int> &result) const {
  int offset = index - facts_->VarBegin(current);

  for (size_t i=current, n=state.size(); i<n; ++i) {
    int next = facts_->Fact(i, state[i]) + offset;
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

void SuccessorGenerator::DFSample(const vector<int> &state, int index,
                                  size_t current, unsigned int *k, int *result) {
  int offset = index - facts_->VarBegin(current);

  for (size_t i=current, n=state.size(); i<n; ++i) {
    int next = facts_->Fact(i, state[i]) + offset;
    int data_index = to_data_[next];

    if (data_index != -1) {
      for (auto d : data_[data_index]) {
        if (engine_() % *k == 0) *result = d;
        ++(*k);
      }
    }

    int child = to_child_[next];
    if (child == -1) continue;

    DFSample(state, child, i + 1, k, result);
  }
}

void SuccessorGenerator::Dump() const {
  std::cout << "to child" << std::endl;

  for (auto v : to_child_)
    std::cout << v << " ";

  std::cout << std::endl;
  std::cout << "to data" << std::endl;

  for (auto v : to_data_)
    std::cout << v << " ";

  std::cout << std::endl;
  std::cout << "data" << std::endl;

  for (auto d : data_) {
    for (auto v : d)
      std::cout << v << " ";

    std::cout << std::endl;
  }
}

} // namespace rwls
