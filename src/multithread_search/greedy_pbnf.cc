#include "multithread_search/greedy_pbnf.h"

#include <algorithm>
#include <iostream>
#include <thread>

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

  int max_abstract_nodes = 50000;

  if (auto opt = pt.get_optional<int>("max_abstract_nodes"))
    max_abstract_nodes = opt.get();

  auto open_list_option = pt.get_child("open_list");

  int closd_exponent = 16;

  if (auto opt = pt.get_optional<int>("closd_exponent"))
    closd_exponent = opt.get();

  abstract_graph_ = ConstructByDTG(problem_, max_abstract_nodes);
  nblocks_.reserve(abstract_graph_->n_nodes());

  for (int i = 0; i< abstract_graph_->n_nodes(); ++i)
    nblocks_.emplace_back(std::make_shared<NBlock>(open_list_option, i,
                                                   closd_exponent));

  if (auto opt = pt.get_optional<int>("min_expansions"))
    min_expansions_ = opt.get();

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
  auto node = new SearchNode();
  node->cost = 0;
  node->action = -1;
  node->parent = nullptr;
  node->packed_state.resize(packer_->block_size());
  packer_->Pack(state, node->packed_state.data());
  node->hash = hash_->operator()(state);

  std::vector<int> values;
  node->h = Evaluate(0, state, node, values);
  std::vector<int> abstract_state(abstract_graph_->vars().size());
  int idx = abstract_graph_->StateToNode(state, abstract_state);
  nblocks_[idx]->Push(values, node, true);
  nblocks_[idx]->Close(node);
  freelist_.Push(nblocks_[idx]);
  std::cout << "Initial heuristic value: " << node->h << std::endl;
  std::cout << "Initial block: " << idx << std::endl;
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
                        SearchNode* node, vector<int> &values) {
  values.clear();

  for (auto e : evaluators_[i]) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}

std::shared_ptr<NBlock> GreedyPBNF::BestScope(std::shared_ptr<NBlock> b) const {
  std::shared_ptr<NBlock> best = nullptr;

  for (auto idx : InterferenceScope(b)) {
    auto bp = nblocks_[idx];

    if (bp->priority() != -1
        && (best == nullptr || bp->priority() < best->priority()))
      best = bp;
  }

  return best;
}

SearchNode* GreedyPBNF::Search() {
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
    b = NextNBlock(i, b);
    int exp = 0;

    while (!ShouldSwitch(i, b, &exp)) {
      auto node = b->Pop();

      ++exp;
      packer_->Unpack(node->packed_state.data(), state);

      if (problem_->IsGoal(state)) {
        WriteGoal(node);
        break;
      }

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

        std::size_t closd_idx = nblocks_[idx]->GetIndex(hash, packed);

        if (nblocks_[idx]->GetItem(closd_idx) != nullptr) continue;

        auto child_node = new SearchNode();
        child_node->cost = node->cost + problem_->ActionCost(o);
        child_node->action = o;
        child_node->parent = node;
        child_node->packed_state = packed;
        child_node->hash = hash;
        ++generated;

        int h = Evaluate(i, child, child_node, values);
        child_node->h = h;
        ++evaluated;

        if (h == -1) {
          ++dead_ends;
          delete child_node;
          continue;
        }

        nblocks_[idx]->Close(closd_idx, child_node);
        node_pool_[i].push_back(child_node);
        bool is_pref = use_preferred_ && preferred.find(o) != preferred.end();
        nblocks_[idx]->Push(values, child_node, is_pref);

        if ((best_h == -1 || h < best_h) && i == 0) {
          best_h = h;
          std::cout << "New best heuristic value: " << best_h << std::endl;
          std::cout << "[" << generated << " generated, "
                    << expanded << " expanded]"  << std::endl;
          std::cout << "best nblock: " << idx << std::endl;
        }
      }
    }
  }

  WriteStat(expanded, evaluated, generated, dead_ends);
}

bool GreedyPBNF::ShouldSwitch(int i, std::shared_ptr<NBlock> b, int *exp) {
  std::unique_lock<std::mutex> lk(mtx_, std::defer_lock);

  if (b == nullptr || b->IsEmpty()) return true;

  if (*exp < min_expansions_) return false;

  *exp = 0;
  auto best_free = freelist_.Minimum();
  auto best_scope = BestScope(b);

  if ((best_free != -1 && best_free < b->priority())
      || (best_scope != nullptr && best_scope->priority() != -1
          && best_scope->priority() < b->priority())) {
    if (best_scope != nullptr && best_scope->priority() != -1
        && (best_free == -1 || best_scope->priority() < best_free)) {
      SetHot(best_scope);
    }

    return true;
  }

  lk.lock();

  for (auto bp : InterferenceScope(b))
    if (nblocks_[bp]->hot()) SetCold(nblocks_[bp]);

  return false;
}

void GreedyPBNF::SetHot(std::shared_ptr<NBlock> b) {
  std::unique_lock<std::mutex> lk(mtx_);

  if (!b->hot() && b->sigma() > 0) {
    bool any = false;

    for (auto idx : InterferenceScope(b)) {
      auto i = nblocks_[idx];

      if (i->priority() != -1
          && (b->priority() == -1 || i->priority() < b->priority())
          && i->hot()) {
        any = true;
        break;
      }
    }

    if (!any) {
      b->set_hot();

      for (auto idx : InterferenceScope(b)) {
        auto mp = nblocks_[idx];

        if (mp->hot()) SetCold(mp);

        if (mp->is_free() && mp->heap_idx() >= 0)
          freelist_.Remove(mp);

        mp->increment_sigma_h();
      }
    }
  }
}

bool GreedyPBNF::SetCold(std::shared_ptr<NBlock> b) {
  bool broadcast = false;
  b->set_cold();

  for (auto idx : InterferenceScope(b)) {
    auto mp = nblocks_[idx];
    mp->decrement_sigma_h();

    if (mp->is_free()) {
      //if (mp->hot()) SetCold(mp);
      freelist_.Push(mp);
      broadcast = true;
    }
  }

  return broadcast;
}

void GreedyPBNF::Release(int i, std::shared_ptr<NBlock> b) {
  b->unuse();

  if (!b->IsEmpty() && b->is_free()) {
    freelist_.Push(b);
    cond_.notify_all();
  }

  bool broadcast = false;

  for (auto idx : InterferenceScope(b)) {
    auto bp = nblocks_[idx];
    bp->decrement_sigma();

    if (bp->sigma() == 0) {
      if (bp->hot()) broadcast |= SetCold(bp);

      if (bp->is_free()) {
        freelist_.Push(bp);
        broadcast = true;
      }
    }
  }

  if (broadcast) cond_.notify_all();
}

std::shared_ptr<NBlock> GreedyPBNF::NextNBlock(int i, std::shared_ptr<NBlock> b) {
  std::unique_lock<std::mutex> lk(mtx_, std::defer_lock);

  if (b == nullptr || b->IsEmpty() || b->hot())
    lk.lock();
  else
    if (!lk.try_lock()) return b;

  if (b != nullptr) {
    auto best_scope = BestScope(b);
    auto best_free = freelist_.Minimum();

    if (b->priority() != -1
        && (best_scope == nullptr || best_scope->priority() == -1
            || b->priority() < best_scope->priority())
        && (best_free == -1 || b->priority() < best_free))
      return b;

    Release(i, b);
  }

  if (freelist_.IsEmpty()) {
    bool all = true;

    for (auto l : nblocks_) {
      if (l->sigma() != 0) {
        all = false;
        break;
      }
    }

    if (all) {
      done_ = true;
      cond_.notify_all();
    }
  }

  if (freelist_.IsEmpty() && !done_)
    cond_.wait(lk, [this]() -> bool {
        return !this->freelist_.IsEmpty() || this->done_;
    });

  std::shared_ptr<NBlock> m = nullptr;

  if (!done_) {
    m = freelist_.Pop();
    m->use();

    for (auto idx : InterferenceScope(m)) {
      if (nblocks_[idx]->is_free() && nblocks_[idx]->heap_idx() >= 0)
        freelist_.Remove(nblocks_[idx]);

      nblocks_[idx]->increment_sigma();
    }

  }

  return m;
}

void GreedyPBNF::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

} // namespace pplanner
