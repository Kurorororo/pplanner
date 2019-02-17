#include "multithread_search/gbfs_portfolio.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "open_list_factory.h"
#include "multithread_search/heuristic_factory.h"

namespace pplanner {

using std::make_shared;
using std::size_t;
using std::unordered_set;
using std::vector;

void GBFSPortfolio::InitHeuristics(int i,
                                   const boost::property_tree::ptree pt) {
  node_pool_[i].reserve(1 << 22);

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    auto evaluator = HeuristicFactory<SearchNode*>(problem_, e);
    evaluators_[i].push_back(evaluator);
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same")
        preferring_[i] = evaluators_[i][0];
      else
        preferring_[i] = HeuristicFactory<SearchNode*>(problem_,
                                                       preferring.get());
    }
  }

  if (i == 0) {
    open_lists_[i] = std::make_shared<SingleOpenList<SearchNodeWithNext*> >(
        "fifo");
  } else if (i == 1) {
    open_lists_[i] = std::make_shared<SingleOpenList<SearchNodeWithNext*> >(
        "lifo");
  } else {
    open_lists_[i] = std::make_shared<SingleOpenList<SearchNodeWithNext*> >(
        "ro");
  }

  if (!share_closed_) closed_lists_[i] = std::make_shared<ClosedList>(22);
}

void GBFSPortfolio::Init(const boost::property_tree::ptree &pt) {
  goal_.store(nullptr);

  if (auto opt = pt.get_optional<int>("n_threads")) n_threads_ = opt.get();

  preferring_.resize(n_threads_);
  evaluators_.resize(n_threads_);
  node_pool_.resize(n_threads_);

  if (auto opt = pt.get_optional<int>("share_closed"))
    share_closed_ = opt.get();

  if (share_closed_) {
    int closed_exponent = 26;

    if (auto opt = pt.get_optional<int>("closed_exponent"))
      closed_exponent = opt.get();

    shared_closed_ = std::make_unique<LockFreeClosedList>(closed_exponent);
  } else {
    closed_lists_.resize(n_threads_, nullptr);
  }

  open_lists_.resize(n_threads_, nullptr);

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i, pt] { this->InitHeuristics(i, pt); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();
}

void GBFSPortfolio::InitialEvaluate(int i) {
  auto state = problem_->initial();
  auto node = new SearchNodeWithNext();
  node->cost = 0;
  node->action = -1;
  node->parent = nullptr;
  node->packed_state.resize(packer_->block_size());
  packer_->Pack(state, node->packed_state.data());
  node->hash = hash_->operator()(state);
  node->next.store(nullptr);

  std::vector<int> values;
  node->h = Evaluate(i, state, node, values);
  open_lists_[i]->Push(values, node, false);
}

void GBFSPortfolio::Distribute() {
  InitialEvaluate(0);

  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<int> values;
  vector<uint32_t> packed(packer_->block_size(), 0);

  int best_h = -1;

  while (!open_lists_[0]->IsEmpty() && open_lists_[0]->size() < n_threads_) {
    auto node = open_lists_[0]->Pop();

    if (share_closed_) {
      if (shared_closed_->IsClosed(node->hash, node->packed_state)) continue;
      shared_closed_->Close(node);
    } else {
      if (!closed_lists_[0]->Close(node)) continue;
    }

    packer_->Unpack(node->packed_state.data(), state);
    ++expanded_;

    if (problem_->IsGoal(state)) {
      WriteGoal(node);
      break;
    }

    generator_->Generate(state, applicable);

    if (applicable.empty()) {
      ++dead_ends_;
      continue;
    }

    if (use_preferred_)
      preferring_[0]->Evaluate(state, applicable, preferred, node);

    for (auto o : applicable) {
      problem_->ApplyEffect(o, state, child);

      uint32_t hash = hash_->HashByDifference(o, node->hash, state, child);
      packer_->Pack(child, packed.data());

      if (share_closed_) {
        if (shared_closed_->IsClosed(hash, packed)) continue;
      } else {
        if (closed_lists_[0]->IsClosed(hash, packed)) continue;
      }

      auto child_node = new SearchNodeWithNext();
      child_node->cost = node->cost + problem_->ActionCost(o);
      child_node->action = o;
      child_node->parent = node;
      child_node->packed_state = packed;
      child_node->hash = hash;
      child_node->next.store(nullptr);
      ++generated_;

      int h = Evaluate(0, child, child_node, values);
      child_node->h = h;
      ++evaluated_;

      if (h == -1) {
        ++dead_ends_;
        delete child_node;
        continue;
      }

      node_pool_[0].push_back(child_node);

      if (best_h == -1 || h < best_h) {
        best_h = h;
        std::cout << "New best heuristic value: " << best_h << std::endl;
        std::cout << "[" << generated_ << " generated, "
                  << expanded_ << " expanded]"  << std::endl;
      }

      bool is_pref = use_preferred_ && preferred.find(o) != preferred.end();
      open_lists_[0]->Push(values, child_node, is_pref);
    }
  }

  if (open_lists_[0]->IsEmpty()) return;

  for (int i = 0; i < n_threads_; ++i) {
    auto values = open_lists_[0]->MinimumValues();
    auto node = open_lists_[0]->Pop();
    open_lists_[i]->Push(values, node, true);
  }
}

void GBFSPortfolio::DeleteAllNodes(int i) {
  for (int j = 0, n = node_pool_[i].size(); j < n; ++j)
    delete node_pool_[i][j];
}

GBFSPortfolio::~GBFSPortfolio() {
  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->DeleteAllNodes(i); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();
}

int GBFSPortfolio::Evaluate(int i, const vector<int> &state,
                        SearchNodeWithNext* node, vector<int> &values) {
  values.clear();

  for (auto e : evaluators_[i]) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}

void GBFSPortfolio::ThreadSearch(int i) {
  if (!share_closed_) InitialEvaluate(i);

  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<int> values;
  vector<uint32_t> packed(packer_->block_size(), 0);

  int best_h = -1;
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;

  while (goal_.load() == nullptr) {
    if (open_lists_[i]->IsEmpty()) continue;

    auto node = open_lists_[i]->Pop();

    if (share_closed_) {
      if (shared_closed_->IsClosed(node->hash, node->packed_state)) continue;
      shared_closed_->Close(node);
    } else {
      if (!closed_lists_[i]->Close(node)) continue;
    }

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
      preferring_[i]->Evaluate(state, applicable, preferred, node);

    for (auto o : applicable) {
      problem_->ApplyEffect(o, state, child);

      uint32_t hash = hash_->HashByDifference(o, node->hash, state, child);
      packer_->Pack(child, packed.data());

      if (share_closed_) {
        if (shared_closed_->IsClosed(hash, packed)) continue;
      } else {
        if (closed_lists_[i]->IsClosed(hash, packed)) continue;
      }

      auto child_node = new SearchNodeWithNext();
      child_node->cost = node->cost + problem_->ActionCost(o);
      child_node->action = o;
      child_node->parent = node;
      child_node->packed_state = packed;
      child_node->hash = hash;
      child_node->next.store(nullptr);
      ++generated;

      int h = Evaluate(i, child, child_node, values);
      child_node->h = h;
      ++evaluated;

      if (h == -1) {
        ++dead_ends;
        delete child_node;
        continue;
      }

      node_pool_[i].push_back(child_node);

      if ((best_h == -1 || h < best_h) && i == 0) {
        best_h = h;
        std::cout << "New best heuristic value: " << best_h << std::endl;
        std::cout << "[" << generated << " generated, "
                  << expanded << " expanded]"  << std::endl;
      }

      bool is_pref = use_preferred_ && preferred.find(o) != preferred.end();
      open_lists_[i]->Push(values, child_node, is_pref);
    }
  }

  WriteStat(expanded, evaluated, generated, dead_ends);
}

SearchNodeWithNext* GBFSPortfolio::Search() {
  if (share_closed_) Distribute();

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->ThreadSearch(i); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();

  return goal_.load();
}

void GBFSPortfolio::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

} // namespace pplanner
