#include "multithread_search/multi_focus_gbfs.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "multithread_search/heuristic_factory.h"

namespace pplanner {

using std::make_shared;
using std::size_t;
using std::unordered_set;
using std::vector;

void MultiFocusGBFS::InitHeuristics(int i,
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
        preferring_[i] = HeuristicFactory<SearchNode*>(
            problem_, preferring.get());
    }
  }
}

void MultiFocusGBFS::Init(const boost::property_tree::ptree &pt) {
  goal_.store(nullptr);
  int closed_exponent = 26;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  open_list_option_ = pt.get_child("open_list");
  foci_ = OpenListFactory<std::shared_ptr<Focus> >(open_list_option_);
  closed_ = std::make_unique<LockFreeClosedList>(closed_exponent);

  if (auto opt = pt.get_optional<int>("n_threads")) n_threads_ = opt.get();

  preferring_.resize(n_threads_);
  evaluators_.resize(n_threads_);
  node_pool_.resize(n_threads_);

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i, pt] { this->InitHeuristics(i, pt); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();
}

void MultiFocusGBFS::InitialEvaluate() {
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
  node->h = Evaluate(0, state, node, values);
  best_h_.store(node->h);
  CreateNewFocus(values, node, true);
  std::cout << "Initial heuristic value: " << node->h << std::endl;
}

void MultiFocusGBFS::DeleteAllNodes(int i) {
  for (int j = 0, n = node_pool_[i].size(); j < n; ++j)
    delete node_pool_[i][j];
}

MultiFocusGBFS::~MultiFocusGBFS() {
  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->DeleteAllNodes(i); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();
}

int MultiFocusGBFS::Evaluate(int i, const vector<int> &state,
                             SearchNodeWithNext *node, vector<int> &values) {
  values.clear();

  for (auto e : evaluators_[i]) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}

bool MultiFocusGBFS::UpdateBestH(int h) {
  int expected = best_h_.load();

  do {
    expected = best_h_.load();

    if (h >= expected) return false;

  } while (!best_h_.compare_exchange_weak(expected, h));

  return true;
}

void MultiFocusGBFS::Expand(int i) {
  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<int> values;
  vector<int> best_values;
  vector<uint32_t> packed(packer_->block_size(), 0);

  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;

  while (goal_ == nullptr) {
    auto focus = LockedPopFocus();

    if (focus == nullptr) continue;

    int counter = min_expansion_per_focus_;

    while (counter > 0 && !focus->IsEmpty()) {
      auto node = focus->Pop();

      if (closed_->IsClosed(node->hash, node->packed_state)) break;

      packer_->Unpack(node->packed_state.data(), state);
      closed_->Close(node);
      ++expanded;

      if (problem_->IsGoal(state)) {
        WriteGoal(node);
        break;
      }

      generator_->Generate(state, applicable);

      if (applicable.empty()) {
        ++dead_ends;
        break;
      }

      if (use_preferred_)
        preferring_[i]->Evaluate(state, applicable, preferred, node);

      SearchNodeWithNext *best_node = nullptr;

      for (auto o : applicable) {
        problem_->ApplyEffect(o, state, child);

        uint32_t hash = hash_->HashByDifference(o, node->hash, state, child);
        packer_->Pack(child, packed.data());

        if (closed_->IsClosed(hash, packed)) continue;

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

        bool is_pref = use_preferred_ && preferred.find(o) != preferred.end();
        focus->Push(values, child_node, is_pref);

        if (h < focus->best_h()) {
          focus->set_best_h(h);
          focus->Boost();

          if (UpdateBestH(h)) {
            best_node = child_node;
            best_values = values;
          }
        }
      }

      if (best_node != nullptr) {
        CreateNewFocus(best_values, best_node, true);
        ++counter;

        if (i == 0) {
          std::cout << "New focus h=" << best_node->h << std::endl;
          std::cout << "New best heuristic value: " << best_node->h
                    << std::endl;
          std::cout << "[" << generated << " generated, " << expanded
                    << " expanded]"  << std::endl;
        }
      }

      --counter;
    }

    if (!focus->IsEmpty()) LockedPushFocus(focus);
  }

  WriteStat(expanded, evaluated, generated, dead_ends);
}

SearchNodeWithNext* MultiFocusGBFS::Search() {
  InitialEvaluate();
  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->Expand(i); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();

  return goal_;
}

void MultiFocusGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

} // namespace pplanner
