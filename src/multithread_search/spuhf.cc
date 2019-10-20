#include "multithread_search/spuhf.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"

namespace pplanner {

using std::make_shared;
using std::shared_ptr;
using std::size_t;
using std::unordered_set;
using std::vector;

void SPUHF::InitHeuristics(int i, const boost::property_tree::ptree pt) {
  auto e = pt.get_child("evaluator");
  evaluators_[i] = EvaluatorFactory(problem_, e);

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same")
        preferring_[i] = evaluators_[i];
      else
        preferring_[i] = EvaluatorFactory(problem_, preferring.get());
    }
  }
}

void SPUHF::Init(const boost::property_tree::ptree& pt) {
  goal_ = nullptr;
  int closed_exponent = 26;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory<int, std::shared_ptr<SearchNodeWithFlag>>(
      open_list_option);
  speculative_list_ = OpenListFactory<int, std::shared_ptr<SearchNodeWithFlag>>(
      open_list_option);
  closed_ =
      std::make_unique<LockFreeClosedList<SearchNodeWithFlag>>(closed_exponent);
  cached_ =
      std::make_unique<LockFreeClosedList<SearchNodeWithFlag>>(closed_exponent);

  if (auto opt = pt.get_optional<int>("n_threads")) n_threads_ = opt.get();

  preferring_.resize(n_threads_);
  evaluators_.resize(n_threads_);

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i, pt] { this->InitHeuristics(i, pt); }));

  for (int i = 0; i < n_threads_; ++i) ts[i].join();
}

void SPUHF::InitialEvaluate() {
  auto state = problem_->initial();
  auto node = std::make_shared<SearchNodeWithFlag>();
  node->cost = 0;
  node->action = -1;
  node->parent = nullptr;
  node->packed_state.resize(packer_->block_size());
  packer_->Pack(state, node->packed_state.data());
  node->hash = hash_->operator()(state);
  node->next = nullptr;
  node->certain = true;

  node->h = evaluators_[0]->Evaluate(state, node);
  open_list_->Push(node->h, node, false);
  std::cout << "Initial heuristic value: " << node->h << std::endl;
}

std::shared_ptr<SPUHF::SearchNodeWithFlag> SPUHF::LockedPop() {
  std::lock_guard<std::mutex> lock(open_mtx_);

  if (open_list_->IsEmpty()) return nullptr;

  if (open_list_->Top()->certain || n_expanding_ == 0) {
    ++n_expanding_;
    return open_list_->Pop();
  }

  return nullptr;
}

std::shared_ptr<SPUHF::SearchNodeWithFlag> SPUHF::SpeculativePop() {
  std::lock_guard<std::mutex> lock(speculative_mtx_);

  if (speculative_list_->IsEmpty()) return nullptr;

  return speculative_list_->Pop();
}

void SPUHF::LockedPush(
    int n, vector<shared_ptr<SPUHF::SearchNodeWithFlag>> node_buffer,
    vector<bool> is_preferred_buffer) {
  std::lock_guard<std::mutex> lock(open_mtx_);

  for (int i = 0; i < n; ++i)
    open_list_->Push(node_buffer[i]->h, node_buffer[i], is_preferred_buffer[i]);

  --n_expanding_;
}

void SPUHF::SpeculativePush(
    int n, vector<shared_ptr<SPUHF::SearchNodeWithFlag>> node_buffer,
    vector<bool> is_preferred_buffer) {
  std::lock_guard<std::mutex> lock(speculative_mtx_);

  for (int i = 0; i < n; ++i)
    speculative_list_->Push(node_buffer[i]->h, node_buffer[i],
                            is_preferred_buffer[i]);
}

void SPUHF::Expand(int i) {
  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<uint32_t> packed(packer_->block_size(), 0);
  vector<std::shared_ptr<SearchNodeWithFlag>> node_buffer;
  vector<bool> is_preferred_buffer;

  int best_h = -1;
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;

  while (goal_ == nullptr) {
    auto node = LockedPop();
    bool speculative = false;

    if (node == nullptr) {
      node = SpeculativePop();
      if (node == nullptr) continue;
      speculative = true;
    }

    if (!closed_->Close(node)) {
      if (!speculative) --n_expanding_;
      continue;
    }

    packer_->Unpack(node->packed_state.data(), state);
    ++expanded;

    if (problem_->IsGoal(state)) {
      WriteGoal(node);
      if (!speculative) --n_expanding_;
      break;
    }

    generator_->Generate(state, applicable);

    if (applicable.empty()) {
      ++dead_ends;
      if (!speculative) --n_expanding_;
      continue;
    }

    if (use_preferred_)
      preferring_[i]->Evaluate(state, node, applicable, preferred);

    int n_children = 0;
    node_buffer.resize(applicable.size(), nullptr);
    is_preferred_buffer.resize(applicable.size());
    int min_h = -1;

    for (auto o : applicable) {
      problem_->ApplyEffect(o, state, child);

      uint32_t hash = hash_->HashByDifference(o, node->hash, state, child);
      packer_->Pack(child, packed.data());

      if (closed_->IsClosed(hash, packed)) continue;

      auto& child_node = node_buffer[n_children];
      auto found = cached_->Find(hash, packed);

      if (found != nullptr) {
        if (speculative) continue;
        child_node = found;
        if (min_h == -1 || child_node->h < min_h) min_h = child_node->h;
      } else {
        child_node = std::make_shared<SearchNodeWithFlag>();
        child_node->cost = node->cost + problem_->ActionCost(o);
        child_node->action = o;
        child_node->parent = node;
        child_node->packed_state = packed;
        child_node->hash = hash;
        child_node->next = nullptr;
        child_node->certain = false;
        ++generated;
        int h = evaluators_[i]->Evaluate(child, child_node);
        child_node->h = h;
        ++evaluated;

        if (h == -1) {
          ++dead_ends;
          continue;
        }

        if (min_h == -1 || h < min_h) min_h = h;

        if ((best_h == -1 || h < best_h) && i == 0) {
          best_h = h;
          std::cout << "New best heuristic value: " << best_h << std::endl;
          std::cout << "[" << generated << " generated, " << expanded
                    << " expanded]" << std::endl;
        }

        if (speculative) cached_->Close(child_node);
      }

      is_preferred_buffer[n_children] =
          use_preferred_ && preferred.find(o) != preferred.end();

      ++n_children;
    }

    if (min_h != -1 && min_h < node->h) {
      for (int j = 0; j < n_children; ++j)
        node_buffer[j]->certain = node_buffer[j]->h == min_h;
    }

    if (n_children > 0 && !speculative)
      LockedPush(n_children, node_buffer, is_preferred_buffer);
    else if (n_children > 0 && speculative)
      SpeculativePush(n_children, node_buffer, is_preferred_buffer);
    else if (!speculative)
      --n_expanding_;
  }

  WriteStat(expanded, evaluated, generated, dead_ends);
}

std::shared_ptr<SPUHF::SearchNodeWithFlag> SPUHF::Search() {
  InitialEvaluate();
  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->Expand(i); }));

  for (int i = 0; i < n_threads_; ++i) ts[i].join();

  return goal_;
}

void SPUHF::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

}  // namespace pplanner