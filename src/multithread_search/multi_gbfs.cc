#include "multithread_search/multi_gbfs.h"

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"
#include "search_graph_factory.h"

namespace pplanner {

using std::make_shared;
using std::size_t;
using std::unordered_set;
using std::vector;

void MultiGBFS::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  if (auto opt = pt.get_optional<int>("n_threads")) n_threads_ = opt.get();

  bool keep_cost = false;
  if (auto opt = pt.get_optional<int>("keep_cost")) keep_cost = true;

  bool use_landmark = false;
  if (auto opt = pt.get_optional<int>("landmark")) use_landmark = true;

  bool dump_nodes = false;
  if (auto opt = pt.get_optional<int>("dump_nodes")) dump_nodes = true;

  graph_ = SearchGraphFactory(
      problem_, closed_exponent, keep_cost, use_landmark, dump_nodes);

  std::shared_ptr<Evaluator> friend_evaluator = nullptr;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    auto evaluator = EvaluatorFactory(problem_, graph_, friend_evaluator, e);
    evaluators_.push_back(evaluator);
    friend_evaluator = evaluator;
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same") {
        preferring_ = evaluators_[0];
      } else {
        preferring_ = EvaluatorFactory(problem_, graph_, nullptr,
                                       preferring.get());
      }
    }
  }

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option, evaluators_);

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram"))
    ram = opt.get();

  graph_->ReserveByRAMSize(ram);
}

void MultiGBFS::InitialExpand() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);

  std::vector<int> values;
  int h = Evaluate(state, node, values);
  graph_->SetH(node, h);
  open_list_->Push(values, node, false);
  std::cout << "Initial heuristic value: " << h << std::endl;
}

int MultiGBFS::Evaluate(const vector<int> &state, int node,
                        vector<int> &values) {
  values.clear();

  for (auto e : evaluators_) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}

void MultiGBFS::Expand() {
  WriteReporter();

  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<int> values;
  vector<uint32_t> packed(graph_->block_size(), 0);

  int best_h = -1;
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;

  while (!LockedReadGoal()) {
    int node = LockedPop();

    if (node == -1) continue;

    auto current_packed = graph_->PackedState(node);
    uint32_t hash = graph_->HashValue(node);

    if (LockedClosedCheck(hash, current_packed)) continue;

    LockedClose(node);

    graph_->Expand(node, state);
    ++expanded;

    if (problem_->IsGoal(state)) {
      LockedWriteGoal(node);
      break;
    }

    generator_->Generate(state, applicable);

    if (applicable.empty()) {
      ++dead_ends;
      continue;
    }

    if (use_preferred_)
      preferring_->Evaluate(state, node, applicable, preferred);

    for (auto o : applicable) {
      child = state;
      problem_->ApplyEffect(o, child);

      uint32_t hash = graph_->HashByDifference(o, node, state, child);
      graph_->Pack(child, packed.data());

      if (LockedClosedCheck(hash, packed.data())) continue;

      int child_node = LockedAllocateNode();
      graph_->WriteNode(child_node, o, node, hash, packed.data());
      ++generated;

      int h = Evaluate(child, child_node, values);
      ++evaluated;
      graph_->SetH(child_node, h);

      if (h == -1) {
        ++dead_ends;
        continue;
      }

      if ((best_h == -1 || h < best_h)
          && reporter_tid_ == std::this_thread::get_id()) {
        best_h = h;
        std::cout << "New best heuristic value: " << best_h << std::endl;
        std::cout << "[" << generated << " generated, "
                  << expanded << " expanded]"  << std::endl;
      }

      bool is_pref = use_preferred_ && preferred.find(o) != preferred.end();

      LockedPush(values, child_node, is_pref);
    }
  }

  WriteStat(expanded, evaluated, generated, dead_ends);
}

int MultiGBFS::Search() {
  InitialExpand();
  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this] { this->Expand(); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();

  return goal_;
}

void MultiGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
  graph_->Dump();
}

} // namespace pplanner
