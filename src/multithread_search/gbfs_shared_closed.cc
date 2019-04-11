#include "multithread_search/gbfs_shared_closed.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"

namespace pplanner {

using std::make_shared;
using std::size_t;
using std::unordered_set;
using std::vector;

void GBFSSharedClosed::InitHeuristics(int i,
                                      const boost::property_tree::ptree pt) {
  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    auto evaluator = EvaluatorFactory(problem_, e);
    evaluators_[i].push_back(evaluator);
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same")
        preferring_[i] = evaluators_[i][0];
      else
        preferring_[i] = EvaluatorFactory(problem_, preferring.get());
    }
  }
}

void GBFSSharedClosed::Init(const boost::property_tree::ptree& pt) {
  goal_ = nullptr;
  int closed_exponent = 26;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  if (auto opt = pt.get_optional<int>("n_threads")) n_threads_ = opt.get();

  auto open_list_option = pt.get_child("open_list");

  for (int i = 0; i < n_threads_; ++i) {
    open_lists_.push_back(
        OpenListFactory<std::vector<int>, std::shared_ptr<SearchNodeWithNext> >(
            open_list_option));
  }

  closed_ = std::make_unique<LockFreeClosedList>(closed_exponent);

  preferring_.resize(n_threads_);
  evaluators_.resize(n_threads_);

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i, pt] { this->InitHeuristics(i, pt); }));

  for (int i = 0; i < n_threads_; ++i) ts[i].join();
}

void GBFSSharedClosed::InitialEvaluate() {
  auto state = problem_->initial();
  auto node = std::make_shared<SearchNodeWithNext>();
  node->cost = 0;
  node->action = -1;
  node->parent = nullptr;
  node->packed_state.resize(packer_->block_size());
  packer_->Pack(state, node->packed_state.data());
  node->hash = hash_->operator()(state);
  node->next = nullptr;

  std::vector<int> values;
  node->h = Evaluate(0, state, node, values);
  open_lists_[0]->Push(values, node, false);
  std::cout << "Initial heuristic value: " << node->h << std::endl;
}

int GBFSSharedClosed::Evaluate(int i, const vector<int>& state,
                               std::shared_ptr<SearchNodeWithNext> node,
                               vector<int>& values) {
  values.clear();

  for (auto e : evaluators_[i]) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}

std::shared_ptr<SearchNodeWithNext> GBFSSharedClosed::GenerateSeeds() {
  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<int> values;
  vector<uint32_t> packed(packer_->block_size(), 0);

  while (open_lists_[0]->size() < n_threads_) {
    if (open_lists_[0]->IsEmpty()) break;

    auto node = open_lists_[0]->Pop();

    if (closed_->IsClosed(node->hash, node->packed_state)) continue;

    packer_->Unpack(node->packed_state.data(), state);
    closed_->Close(node);
    ++expanded_;

    if (problem_->IsGoal(state)) return node;

    generator_->Generate(state, applicable);

    if (applicable.empty()) {
      ++dead_ends_;
      continue;
    }

    if (use_preferred_)
      preferring_[0]->Evaluate(state, node, applicable, preferred);

    for (auto o : applicable) {
      problem_->ApplyEffect(o, state, child);

      uint32_t hash = hash_->HashByDifference(o, node->hash, state, child);
      packer_->Pack(child, packed.data());

      if (closed_->IsClosed(hash, packed)) continue;

      auto child_node = std::make_shared<SearchNodeWithNext>();
      child_node->cost = node->cost + problem_->ActionCost(o);
      child_node->action = o;
      child_node->parent = node;
      child_node->packed_state = packed;
      child_node->hash = hash;
      child_node->next = nullptr;
      ++generated_;

      int h = Evaluate(0, child, child_node, values);
      child_node->h = h;
      ++evaluated_;

      if (h == -1) {
        ++dead_ends_;
        continue;
      }

      if (best_h_ == -1 || h < best_h_) {
        best_h_ = h;
        std::cout << "New best heuristic value: " << best_h_ << std::endl;
        std::cout << "[" << generated_ << " generated, " << expanded_
                  << " expanded]" << std::endl;
      }

      bool is_pref = use_preferred_ && preferred.find(o) != preferred.end();

      open_lists_[0]->Push(values, child_node, is_pref);
    }
  }

  return nullptr;
}

void GBFSSharedClosed::Expand(int i) {
  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<int> values;
  vector<uint32_t> packed(packer_->block_size(), 0);

  int best_h = best_h_;
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;

  while (goal_ == nullptr) {
    if (open_lists_[i]->IsEmpty()) break;

    auto node = open_lists_[i]->Pop();

    if (!closed_->Close(node)) continue;

    packer_->Unpack(node->packed_state.data(), state);
    ++expanded;

    if (problem_->IsGoal(state)) {
      WriteGoal(node);
      break;
    }

    generator_->Generate(state, applicable);

    if (applicable.empty()) {
      ++dead_ends;
      continue;
    }

    if (use_preferred_)
      preferring_[i]->Evaluate(state, node, applicable, preferred);

    for (auto o : applicable) {
      problem_->ApplyEffect(o, state, child);

      uint32_t hash = hash_->HashByDifference(o, node->hash, state, child);
      packer_->Pack(child, packed.data());

      if (closed_->IsClosed(hash, packed)) continue;

      auto child_node = std::make_shared<SearchNodeWithNext>();
      child_node->cost = node->cost + problem_->ActionCost(o);
      child_node->action = o;
      child_node->parent = node;
      child_node->packed_state = packed;
      child_node->hash = hash;
      child_node->next = nullptr;
      ++generated;

      int h = Evaluate(i, child, child_node, values);
      child_node->h = h;
      ++evaluated;

      if (h == -1) {
        ++dead_ends;
        continue;
      }

      if ((best_h == -1 || h < best_h) && i == 0) {
        best_h = h;
        std::cout << "New best heuristic value: " << best_h << std::endl;
        std::cout << "[" << generated << " generated, " << expanded
                  << " expanded]" << std::endl;
      }

      bool is_pref = use_preferred_ && preferred.find(o) != preferred.end();

      open_lists_[i]->Push(values, child_node, is_pref);
    }
  }

  WriteStat(expanded, evaluated, generated, dead_ends);
}

std::shared_ptr<SearchNodeWithNext> GBFSSharedClosed::Search() {
  InitialEvaluate();
  auto goal = GenerateSeeds();

  if (goal != nullptr) return goal;

  for (int i = 1; i < n_threads_; ++i) {
    std::vector<int> values = open_lists_[0]->MinimumValue();
    auto node = open_lists_[0]->Pop();
    open_lists_[i]->Push(values, node, false);
  }

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->Expand(i); }));

  for (int i = 0; i < n_threads_; ++i) ts[i].join();

  return goal_;
}

void GBFSSharedClosed::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

}  // namespace pplanner