#include "multithread_search/greedy_pbnfs.h"

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

void GreedyPBNF::InitHeuristics(int i, const boost::property_tree::ptree pt) {
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
}

void GreedyPBNF::Init(const boost::property_tree::ptree &pt) {
  goal_.store(nullptr);

  if (auto opt = pt.get_optional<int>("max_abstract_nodes"))
    n_abstract_nodes_ = opt.get();

  auto open_list_option = pt.get_child("open_list");

  int closd_exponent = 16;

  if (auto opt = pt.get_optional<int>("closd_exponent"))
    closd_exponent = opt.get();

  abstract_graph_ = ConstructByDTG(problem, max_abstract_nodes_);
  nblocks_.reserve(abstract_graph_->n_nodes());

  for (int i = 0; i< abstract_graph_->n_nodes(); ++i)
    nblocks_.emplace_back(std::make_shared<NBlock>(open_list_option, i,
                                                   closd_exponent));

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

void GreedyPBNF::InitialEvaluate() {
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
  node->h = Evaluate(0, state, node, abstract_state);
  std::vector<int> abstract_state(abstract_graph_->vars().size());
  int idx = abstract_graph_->StateToNode(state, abstract_state);
  nblocks_[i]->Push(values, node, true);
  std::cout << "Initial heuristic value: " << node->h << std::endl;
}

void GreedyPBNF::DeleteAllNodes(int i) {
  for (int j = 0, n = node_pool_[i].size(); j < n; ++j)
    delete node_pool_[i][j];
}

GreedyPBNF::~GreedyPBNF() {
  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->DeleteAllNodes(i); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();
}

int GreedyPBNF::Evaluate(int i, const vector<int> &state,
                        SearchNodeWithNext* node, vector<int> &values) {
  values.clear();

  for (auto e : evaluators_[i]) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}

std::shared_ptr<NBlock> GreedyPBNF::BestScope(std::shared_ptr<NBlock> b) {
  std::shared_ptr<NBlock> best = nullptr;

  for (auto bp : InterferenceScope(b)) {
    if (!bp->IsEmpty()
        && (best == nullptr || bp->MinimumVaues() < best->MinimumVaues()))
      best = bp;
  }

  return best;
}

std::shared_ptr<NBlock> GreedyPBNF::BestFree() {
  while (!freelist_.empty()) {
    if (freelist_.top()->sigma() == 0 && freelist_.top()->hot() == 0)
      freelist_.pop();
    else
      return freelist_.top();
  }

  return nullptr;
}

SearchNodeWithNext* GreedyPBNF::Search() {
  InitialEvaluate();
  vector<std::thread> ts;

  for (int i = 0; i < n_threads_; ++i)
    ts.push_back(std::thread([this, i] { this->ThreadSearch(i); }));

  for (int i = 0; i < n_threads_; ++i)
    ts[i].join();

  return goal_.load();
}

void GreedyPBNF::ThreadSearch(int i) {
  vector<int> state(problem_->n_variables());
  vector<int> child(problem_->n_variables());
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<int> values;
  vector<uint32_t> packed(packer_->block_size(), 0);
  vector<int> abstract_state(abstract_graph_->vars().size());

  int best_h = -1;
  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int dead_ends = 0;

  std::shared_ptr<NBlock> b = nullptr;

  while (!done_) {
    b = NextNBlock();
    int exp = 0;

    while (!ShouldSwitch(b, exp)) {
      auto node = b->Pop();
      packer_->Unpack(node->packed_state.data(), state);

      if (problem_->IsGoal(state)) {
        WriteGoal(node);
        break;
      }

      if (b->IsClosed(node->hash, node->packed_state)) continue;

      b->Close(node);
      ++expanded;

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
        int idx = abstract_graph_->StateToNode(child, abstract_state);
        packer_->Pack(child, packed.data());

        if (nblocks_[idx]->IsClosed(hash, packed)) continue;

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
        nblocks_[idx]->Push(valeus, node, is_pref);
      }
    }
  }

  WriteStat(expanded, evaluated, generated, dead_ends);
}

bool GreedyPBNF::ShouldSwitch(std::shared_ptr<NBlock> b, int &exp) {
  if (b->IsEmpty()) return true;

  if (exp < min_expansions_) return false;

  exp = 0;
  auto best_free = BestFree();
  auto best_scope = BestScope(b);

  if (best_free == nullptr
      || best_scope == nullptr
      || b->MinimumVaues() < best_free->MinimumVaues()
      || b->MinimumVaues() < best_scope->MinimumVaues()) {
    if (best_scope != nullptr
        && (best_free == nullptr
            || best_scope->MinimumVaues() < best_free->MinimumVaues()))
      SetHot(best_scope);

    return true;
  }

  mtx_.lock();

  for (auto bp : InterferenceScope(b))
    if (bp->hot()) SetCold(bp);

  mtx_.unlock();

  return false;
}

void GreedyPBNF::SetHot(std::shared_ptr<NBlock> b) {
  mtx_.lock();

  if (!b->hot() && b->sigma() > 0) {
    bool any = false;

    for (auto i : InterferenceScope(b)) {
      if (!i->IsEmpty()
          && (b->IsEmpty() || i->MinimumVaues() < b->MinimumVaues())
          && i->hot()) {
      }
    }

    if (!any) {
      b->set_hot();

      for (auto mp : InferenceScope(b)) {
        if (mp->hot()) mp->set_cold();

        if (mp->sigma() == 0 && mp->sigma_h() == 0 && !mp->IsEmpty());

        mp->decrement_sigma_h();
      }
    }
  }

  mtx_.unlock();
}

void GreedyPBNF::SetCold(std::shared_ptr<NBlock> b) {
  b->set_cold();

  for (auto mp : InterferenceScope(b)) {
    mp->decrement_sigma_h();

    if (mp->sigma() == 0 && mp->sigma_h() == 0 && !mp->IsEmpty()) {
      if (mp->hot()) {
        SetCold(mp);
        freelist_.push(mp);
      }

      WakeAllSleepingThreads();
    }
  }
}

void GreedyPBNF::Release(std::shared_ptr<NBlock> b) {
  for (auto bp : InterferenceScope(b)) {
    bp->decrement_sigma();

    if (bp->sigma() == 0 && bp->sigma_h() == 0 && !bp->IsEmpty()) {
      if (bp->hot()) {
        SetCold(bp);
        freelist_.push(bp);
      }

      WakeAllSleepingThreads();
    }
  }
}

std::shared_ptr<NBlock> GreedyPBNF::NextNBlock(std::shared_ptr<NBlock> b) {
  if (b == nullptr || b->IsEmpty() || b->IsHot())
    mtx_.lock();
  else
    if (!mtx_.try_lock()) return b;

  if (b != nullptr) {
    auto best_scope = BestScope(b);
    auto best_free = BestFree();

    if ((best_scope == nullptr
         || b->MinimumVaues() < best_scope->MinimumVaues())
        && (best_free == nullptr
         || b->MinimumVaues() < best_free->MinimumVaues())) {
      mtx_.unlock();

      return b;
    }

    if (freelist_.empty()) {
      bool all = true;

      for (auto l : nblocks_) {
        if (l->sigma() != 0) {
          all = false;
          break;
        }
      }

      done_ = true;
      WakeAllSleepingThreads();
    }
  }

  Sleep();

  std::shared_ptr<NBlock> m = nullptr;

  if (!done_) {
    m = BestFree();

    for (auto bp : InterferenceScope(m))
      bp->increment_sigma();
  }

  mtx_.unlock();

  return m;
}

void GreedyPBNF::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

} // namespace pplanner
