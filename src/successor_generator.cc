#include "successor_generator.h"

#include <algorithm>
#include <iostream>
#include <random>

using std::pair;
using std::size_t;
using std::vector;

namespace pplanner {

void SuccessorGenerator::Init(std::shared_ptr<const SASPlus> problem) {
  std::random_device seed_gen;
  engine_ = std::mt19937(seed_gen());

  problem_ = problem;

  to_child_.resize(problem->n_facts(), -1);

  vector<pair<int, int> > precondition;
  vector<int> to_data(problem->n_facts(), -1);
  vector<vector<int> > data;

  for (int i=0, n=static_cast<int>(problem->n_actions()); i<n; ++i)
    Insert(i, precondition, to_data, data);

  ConvertToData(to_data, data);
}

void SuccessorGenerator::Insert(int query,
                                vector<pair<int, int> > &precondition,
                                vector<int> &to_data,
                                vector<vector<int> > &data) {
  assert(nullptr != problem_);

  problem_->CopyPrecondition(query, precondition);
  std::sort(precondition.begin(), precondition.end());

  int n_facts = problem_->n_facts();
  int offset = 0;
  int n_ommited = 0;

  for (int i=0, n=precondition.size(); i<n; ++i) {
    int var = precondition[i].first;
    int value = precondition[i].second;
    int index = offset + problem_->Fact(var, value) - n_ommited;

    if (i == n - 1) {
      AddQuery(index, query, to_data, data);
      return;
    }

    n_ommited = problem_->VarBegin(var + 1);

    if (to_child_[index] == -1) {
      size_t old_size = to_child_.size();
      to_child_[index] = static_cast<int>(old_size);
      size_t new_size = old_size + static_cast<size_t>(n_facts - n_ommited);
      to_child_.resize(new_size, -1);
      to_data.resize(new_size, -1);
    }

    offset = to_child_[index];
  }
}

void SuccessorGenerator::AddQuery(int index, int query, vector<int> &to_data,
                                  vector<vector<int> > &data) {
  if (to_data[index] == -1) {
    size_t old_size = data.size();
    data.resize(old_size + 1);
    to_data[index] = static_cast<int>(old_size);
  }

  int data_index = to_data[index];
  data[data_index].push_back(query);
}

void SuccessorGenerator::ConvertToData(const vector<int> &to_data,
                                       const vector<vector<int> > &data) {
  to_data_.reserve(1 + to_data.size());

  size_t size = 0;

  for (auto &v : data)
    size += v.size();

  data_.reserve(size);

  for (auto &v : to_data) {
    if (v == -1) {
      to_data_.push_back(to_data_.back() + 0);
    } else {
      to_data_.push_back(to_data_.back() + data[v].size());

      for (auto d : data[v])
        data_.push_back(d);
    }
  }
}

void SuccessorGenerator::DFS(const vector<int> &state, int index, int current,
                             vector<int> &result) const {
  int offset = index - problem_->VarBegin(current);

  for (int i=current, n=n_variables_; i<n; ++i) {
    int next = problem_->Fact(i, state[i]) + offset;

    for (int j=to_data_[next], m=to_data_[next + 1]; j<m; ++j)
      result.push_back(data_[j]);

    int child = to_child_[next];
    if (child == -1) continue;

    DFS(state, child, i + 1, result);
  }
}

void SuccessorGenerator::DFSample(const vector<int> &state, int index,
                                  int current, unsigned int &k, int &result) {
  int offset = index - problem_->VarBegin(current);

  for (int i=current, n=n_variables_; i<n; ++i) {
    int next = problem_->Fact(i, state[i]) + offset;

    for (int j=to_data_[next], m=to_data_[next + 1]; j<m; ++j) {
      if (engine_() % k == 0) result = data_[j];
      ++k;
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

  for (auto v : data_)
    std::cout << v << " ";

  std::cout << std::endl;
}

} // namespace rwls
