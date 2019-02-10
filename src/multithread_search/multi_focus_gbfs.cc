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
  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    auto evaluator = HeuristicFactory(problem_, e);
    evaluators_[i].push_back(evaluator);
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same")
        preferring_[i] = evaluators_[i][0];
      else
        preferring_[i] = HeuristicFactory(problem_, preferring.get());
    }
  }
}

void MultiFocusGBFS::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  open_list_option_ = pt.get_child("open_list");
  foci_ = OpenListFactory<std::shared_ptr<Focus> >(open_list_option_);

  if (auto opt = pt.get_optional<int>("n_threads")) n_threads_ = opt.get();

  for (int i = 0; i < n_threads_ * 2; ++i)
    closed_.emplace_back(std::make_shared<ClosedList>(closed_exponent));

  for (int i = 0; i < n_threads_ * 2; ++i)
    closed_mtx_.emplace_back(std::make_unique<std::shared_timed_mutex>());

  preferring_.resize(n_threads_);
  evaluators_.resize(n_threads_);

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i, pt] { this->InitHeuristics(i, pt); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();
}

void MultiFocusGBFS::InitialEvaluate() {
  auto state = problem_->initial();
  auto node = std::make_shared<SearchNodeWithHash>();
  node->cost = 0;
  node->action = -1;
  node->parent = nullptr;
  node->packed_state.resize(packer_->block_size());
  packer_->Pack(state, node->packed_state.data());
  node->hash = hash1_->operator()(state);
  node->hash2 = hash2_->operator()(state);

  std::vector<int> values;
  node->h = Evaluate(0, state, node, values);
  CreateNewFocus(values, node, true);
  std::cout << "Initial heuristic value: " << node->h << std::endl;
}

int MultiFocusGBFS::Evaluate(int i, const vector<int> &state,
                             std::shared_ptr<SearchNodeWithHash> node,
                             vector<int> &values) {
  values.clear();

  for (auto e : evaluators_[i]) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}

void MultiFocusGBFS::Expand(int i) {
  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<uint32_t> packed(packer_->block_size(), 0);
  vector<vector<int> > values;
  vector<std::shared_ptr<SearchNodeWithHash> > nodes;
  vector<bool> is_pref_vec;

  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;

  while (!LockedReadGoal()) {
    auto focus = LockedPopFocus();

    if (focus == nullptr) continue;

    int counter = min_expansion_per_focus_;

    while (counter > 0 && !focus->IsEmpty()) {
      auto node = focus->Pop();
      packer_->Unpack(node->packed_state.data(), state);
      int idx = node->hash2 % (n_threads_ * 2);

      if (LockedIsClosed(idx, node->hash, node->packed_state))
        break;

      LockedClose(idx, node);

      ++expanded;

      if (problem_->IsGoal(state)) {
        LockedWriteGoal(node);
        break;
      }

      generator_->Generate(state, applicable);

      if (applicable.empty()) {
        ++dead_ends;
        break;
      }

      if (use_preferred_)
        preferring_[i]->Evaluate(state, applicable, preferred, node);

      values.resize(applicable.size());
      nodes.resize(applicable.size());
      is_pref_vec.resize(applicable.size());
      int arg_best = -1;
      int c = 0;

      for (auto o : applicable) {
        problem_->ApplyEffect(o, state, child);

        uint32_t hash1 = hash1_->HashByDifference(o, node->hash, state, child);
        packer_->Pack(child, packed.data());
        uint32_t hash2 = hash2_->HashByDifference(o, node->hash2, state, child);
        int idx = hash2 % (n_threads_ * 2);

        if (LockedIsClosed(idx, hash1, packed)) continue;

        auto child_node = std::make_shared<SearchNodeWithHash>();
        child_node->cost = node->cost + problem_->ActionCost(o);
        child_node->action = o;
        child_node->parent = node;
        child_node->packed_state = packed;
        child_node->hash = hash1;
        child_node->hash2 = hash2;
        ++generated;

        int h = Evaluate(i, child, child_node, values[c]);
        child_node->h = h;
        ++evaluated;

        if (h == -1) {
          ++dead_ends;
          continue;
        }

        bool is_pref = use_preferred_ && preferred.find(o) != preferred.end();
        nodes[c] = child_node;
        is_pref_vec[c] = is_pref;

        if (h < focus->best_h()) {
          focus->set_best_h(h);
          arg_best = c;
          focus->Boost();
        }

        ++c;
      }

      for (int k = 0; k < c; ++k) {
        if (k == arg_best) {
          CreateNewFocus(values[k], nodes[k], true);

          if (i == 0) {
            std::cout << "New best heuristic value: " << nodes[k]->h
                      << std::endl;
            std::cout << "[" << generated << " generated, " << expanded
                      << " expanded]"  << std::endl;
          }
        } else {
          focus->Push(values[k], nodes[k], is_pref_vec[k]);
        }
      }

      --counter;
    }

    if (!focus->IsEmpty()) LockedPushFocus(focus);
  }

  WriteStat(expanded, evaluated, generated, dead_ends);
}

std::shared_ptr<SearchNodeWithHash> MultiFocusGBFS::Search() {
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
