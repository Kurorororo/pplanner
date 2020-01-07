#include "multithread_search/pgbfs.h"

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

void PGBFS::InitHeuristics(int i, const boost::property_tree::ptree pt) {
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

void PGBFS::Init(const boost::property_tree::ptree& pt) {
  if (auto opt = pt.get_optional<int>("n_threads")) n_threads_ = opt.get();
  if (auto opt = pt.get_optional<bool>("dump")) dump_ = opt.get();

  int shared_exponent = 26;

  if (auto opt = pt.get_optional<int>("closed_exponent"))
    shared_exponent = opt.get();

  if (auto opt = pt.get_optional<bool>("share_closed")) {
    if (opt.get())
      shared_closed_ = std::make_unique<LockFreeClosedList<SearchNodeWithNext>>(
          shared_exponent);
  }

  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  if (shared_closed_ == nullptr) {
    for (int i = 0; i < n_threads_; ++i)
      closed_lists_.push_back(std::make_shared<ClosedList>(closed_exponent));
  }

  if (auto opt = pt.get_optional<bool>("cache_evaluation")) {
    if (opt.get())
      cached_ = std::make_unique<LockFreeClosedList<SearchNodeWithNext>>(
          shared_exponent);
  }

  std::string all_tie_breaking = "none";

  if (auto opt = pt.get_optional<std::string>("all_tie_breaking"))
    all_tie_breaking = opt.get();

  for (int i = 0; i < n_threads_; ++i) {
    auto open_list_option = pt.get_child("open_list");

    if (all_tie_breaking == "none") {
      if (i == 0) open_list_option.put("tie_breaking", "fifo");
      if (i == 1) open_list_option.put("tie_breaking", "lifo");
      if (i > 1) open_list_option.put("tie_breaking", "ro");
    } else {
      open_list_option.put("tie_breaking", all_tie_breaking);
    }

    open_lists_.push_back(
        OpenListFactory<int, std::shared_ptr<SearchNodeWithNext>>(
            open_list_option));
  }

  preferring_.resize(n_threads_);
  evaluators_.resize(n_threads_);

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i, pt] { this->InitHeuristics(i, pt); }));

  for (int i = 0; i < n_threads_; ++i) ts[i].join();
}

void PGBFS::InitialEvaluate() {
  auto state = problem_->initial();
  auto node = std::make_shared<SearchNodeWithNext>();
  node->cost = 0;
  node->action = -1;
  node->parent = nullptr;
  node->packed_state.resize(packer_->block_size());
  packer_->Pack(state, node->packed_state.data());
  node->hash = hash_->operator()(state);
  node->next = nullptr;

  node->h = evaluators_[0]->Evaluate(state, node);

  if (shared_closed_ == nullptr) {
    for (int i = 0; i < n_threads_; ++i)
      open_lists_[i]->Push(node->h, node, false);
  } else {
    open_lists_[0]->Push(node->h, node, false);
  }

  std::cout << "Initial heuristic value: " << node->h << std::endl;
}

void PGBFS::GenerateSeed() {
  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<uint32_t> packed(packer_->block_size(), 0);

  int best_h = -1;

  while (!open_lists_[0]->IsEmpty() && goal_ == nullptr &&
         static_cast<int>(open_lists_[0]->size()) < n_threads_) {
    auto node = open_lists_[0]->Pop();

    if (!shared_closed_->Close(node)) continue;

    packer_->Unpack(node->packed_state.data(), state);
    ++expanded_;

    if (dump_) {
      std::lock_guard<std::mutex> lock(stat_mtx_);
      expanded_nodes_.push_back(node);
    }

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
      preferring_[0]->Evaluate(state, node, applicable, preferred);

    for (auto o : applicable) {
      problem_->ApplyEffect(o, state, child);

      uint32_t hash = hash_->HashByDifference(o, node->hash, state, child);
      packer_->Pack(child, packed.data());

      if (shared_closed_->IsClosed(hash, packed)) continue;

      std::shared_ptr<SearchNodeWithNext> found =
          cached_ != nullptr ? cached_->Find(hash, packed) : nullptr;

      if (found != nullptr && found->h == -1) continue;

      std::shared_ptr<SearchNodeWithNext> child_node =
          std::make_shared<SearchNodeWithNext>();
      child_node->cost = node->cost + problem_->ActionCost(o);
      child_node->action = o;
      child_node->parent = node;
      child_node->packed_state = packed;
      child_node->hash = hash;
      child_node->next = nullptr;
      ++generated_;

      if (found != nullptr) {
        ++n_cached_;
        child_node->h = found->h;
      } else {
        int h = evaluators_[0]->Evaluate(child, child_node);
        child_node->h = h;
        ++evaluated_;

        if (cached_ != nullptr) cached_->Close(child_node);

        if (h == -1) {
          ++dead_ends_;
          continue;
        }

        if (best_h == -1 || h < best_h) {
          best_h = h;
          std::cout << "New best heuristic value: " << best_h << std::endl;
          std::cout << "[" << generated_ << " generated, " << expanded_
                    << " expanded]" << std::endl;
        }
      }

      bool is_preferred =
          use_preferred_ && preferred.find(o) != preferred.end();

      open_lists_[0]->Push(child_node->h, child_node, is_preferred);
    }
  }
}

void PGBFS::Expand(int i) {
  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<uint32_t> packed(packer_->block_size(), 0);

  int best_h = -1;
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;
  int n_cached = 0;

  while (!open_lists_[i]->IsEmpty() && goal_ == nullptr) {
    auto node = open_lists_[i]->Pop();

    if (shared_closed_ == nullptr) {
      if (!closed_lists_[i]->Close(node)) continue;
    } else {
      if (!shared_closed_->Close(node)) continue;
    }

    packer_->Unpack(node->packed_state.data(), state);
    ++expanded;

    if (dump_) {
      std::lock_guard<std::mutex> lock(stat_mtx_);
      expanded_nodes_.push_back(node);
    }

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

      if (shared_closed_ == nullptr) {
        if (closed_lists_[i]->IsClosed(hash, packed)) continue;
      } else {
        if (shared_closed_->IsClosed(hash, packed)) continue;
      }

      std::shared_ptr<SearchNodeWithNext> found =
          cached_ != nullptr ? cached_->Find(hash, packed) : nullptr;

      if (found != nullptr && found->h == -1) continue;

      std::shared_ptr<SearchNodeWithNext> child_node =
          std::make_shared<SearchNodeWithNext>();
      child_node->cost = node->cost + problem_->ActionCost(o);
      child_node->action = o;
      child_node->parent = node;
      child_node->packed_state = packed;
      child_node->hash = hash;
      child_node->next = nullptr;

      if (found != nullptr) {
        ++n_cached;
        child_node->h = found->h;
      } else {
        ++generated;
        int h = evaluators_[i]->Evaluate(child, child_node);
        child_node->h = h;
        ++evaluated;

        if (cached_ != nullptr) {
          std::shared_ptr<SearchNodeWithNext> cache_node =
              std::make_shared<SearchNodeWithNext>();
          cache_node->h = h;
          cache_node->cost = node->cost + problem_->ActionCost(o);
          cache_node->action = o;
          cache_node->parent = node;
          cache_node->packed_state = packed;
          cache_node->hash = hash;
          cache_node->next = nullptr;
          cached_->Close(cache_node);
        }

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
      }

      bool is_preferred =
          use_preferred_ && preferred.find(o) != preferred.end();

      open_lists_[i]->Push(child_node->h, child_node, is_preferred);
    }
  }

  WriteStat(expanded, evaluated, generated, dead_ends, n_cached);
}

std::shared_ptr<SearchNode> PGBFS::Search() {
  InitialEvaluate();

  if (shared_closed_ != nullptr) {
    GenerateSeed();

    if (goal_ != nullptr) return goal_;

    for (int i = 1; i < n_threads_; ++i) {
      int h = open_lists_[0]->MinimumValue();
      auto node = open_lists_[0]->Pop();
      open_lists_[i]->Push(h, node, false);
    }
  }

  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->Expand(i); }));

  for (int i = 0; i < n_threads_; ++i) ts[i].join();

  return goal_;
}

void PGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
  std::cout << "Cached " << n_cached_ << " state(s)" << std::endl;

  if (dump_) {
    std::ofstream dump_file;
    dump_file.open("expanded_nodes.csv", std::ios::out);

    dump_file << "node_id,parent_node_id,h,dummy1,dummy2,dummy3";

    for (int i = 0; i < problem_->n_variables(); ++i) {
      dump_file << ",v" << i;
    }

    dump_file << std::endl;

    std::vector<int> state(problem_->n_variables());
    int order = 0;

    for (auto node : expanded_nodes_) {
      packer_->Unpack(node->packed_state.data(), state);

      node->id = order;
      int parent_id = -1;

      if (node->parent != nullptr) parent_id = node->parent->id;

      dump_file << order << "," << parent_id << "," << node->h;
      dump_file << ",0,0,0";

      for (int j = 0; j < problem_->n_variables(); ++j)
        dump_file << "," << state[j];

      dump_file << std::endl;
      order += 1;
    }
  }
}

}  // namespace pplanner
