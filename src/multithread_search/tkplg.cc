#include "multithread_search/tkplg.h"

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

void TKPLG::InitHeuristics(int i, const boost::property_tree::ptree pt) {
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

void TKPLG::Init(const boost::property_tree::ptree& pt) {
  goal_ = nullptr;
  int closed_exponent = 26;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  auto open_list_option = pt.get_child("open_list");
  open_list_ =
      OpenListFactory<std::vector<int>, std::shared_ptr<SearchNodeWithNext> >(
          open_list_option);
  closed_ = std::make_unique<LockFreeClosedList>(closed_exponent);

  if (auto opt = pt.get_optional<int>("n_threads")) n_threads_ = opt.get();

  preferring_.resize(n_threads_);
  evaluators_.resize(n_threads_);

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i, pt] { this->InitHeuristics(i, pt); }));

  for (int i = 0; i < n_threads_; ++i) ts[i].join();
}

void TKPLG::InitialEvaluate() {
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
  open_list_->Push(values, node, false);
  std::cout << "Initial heuristic value: " << node->h << std::endl;
}

int TKPLG::Evaluate(int i, const vector<int>& state,
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

std::shared_ptr<SearchNodeWithNext> TKPLG::LockedPop() {
  std::lock_guard<std::mutex> lock(open_mtx_);

  if (open_list_->IsEmpty()) return nullptr;

  int h = open_list_->MinimumValue()[0];
  int n_expanding = n_expanding_.load();
  int h_expanding = h_expanding_.load();

  if (n_expanding > 0 && h > h_expanding) return nullptr;

  if (n_expanding == 0 || h < h_expanding) h_expanding_.store(h);

  ++n_expanding_;

  return open_list_->Pop();
}

void TKPLG::LockedPush(int n, const vector<vector<int> >& values_buffer,
                       vector<shared_ptr<SearchNodeWithNext> > node_buffer,
                       vector<bool> is_preferred_buffer, int next) {
  std::lock_guard<std::mutex> lock(open_mtx_);

  for (int i = 0; i < n; ++i) {
    if (i != next)
      open_list_->Push(values_buffer[i], node_buffer[i],
                       is_preferred_buffer[i]);
  }

  --n_expanding_;
}

void TKPLG::Expand(int i) {
  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<uint32_t> packed(packer_->block_size(), 0);
  // vector<int> values;
  vector<vector<int> > values_buffer;
  vector<std::shared_ptr<SearchNodeWithNext> > node_buffer;
  vector<bool> is_preferred_buffer;

  int best_h = -1;
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;
  int next = -1;

  while (goal_ == nullptr) {
    std::shared_ptr<SearchNodeWithNext> node = nullptr;

    if (next == -1)
      node = LockedPop();
    else
      node = node_buffer[next];

    if (node == nullptr) continue;

    if (!closed_->Close(node)) {
      --n_expanding_;
      continue;
    }

    packer_->Unpack(node->packed_state.data(), state);
    ++expanded;

    if (problem_->IsGoal(state)) {
      WriteGoal(node);
      --n_expanding_;
      break;
    }

    generator_->Generate(state, applicable);

    if (applicable.empty()) {
      ++dead_ends;
      --n_expanding_;
      continue;
    }

    if (use_preferred_)
      preferring_[i]->Evaluate(state, node, applicable, preferred);

    int n_children = 0;
    values_buffer.resize(applicable.size());
    node_buffer.resize(applicable.size(), nullptr);
    is_preferred_buffer.resize(applicable.size());
    next = -1;
    int next_h = node->h;

    for (auto o : applicable) {
      problem_->ApplyEffect(o, state, child);

      uint32_t hash = hash_->HashByDifference(o, node->hash, state, child);
      packer_->Pack(child, packed.data());

      if (closed_->IsClosed(hash, packed)) continue;

      auto& child_node = node_buffer[n_children];
      child_node = std::make_shared<SearchNodeWithNext>();
      child_node->cost = node->cost + problem_->ActionCost(o);
      child_node->action = o;
      child_node->parent = node;
      child_node->packed_state = packed;
      child_node->hash = hash;
      child_node->next = nullptr;
      ++generated;

      auto& values = values_buffer[n_children];
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

      if (h < next_h) {
        next_h = h;
        next = n_children;
      }

      is_preferred_buffer[n_children] =
          use_preferred_ && preferred.find(o) != preferred.end();

      ++n_children;
    }

    if (n_children > 0)
      LockedPush(n_children, values_buffer, node_buffer, is_preferred_buffer,
                 next);
    else
      --n_expanding_;
  }

  WriteStat(expanded, evaluated, generated, dead_ends);
}

std::shared_ptr<SearchNodeWithNext> TKPLG::Search() {
  InitialEvaluate();
  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->Expand(i); }));

  for (int i = 0; i < n_threads_; ++i) ts[i].join();

  return goal_;
}

void TKPLG::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

}  // namespace pplanner