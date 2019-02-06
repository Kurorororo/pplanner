#include "multi_focus_gbfs.h"

#include <array>
#include <iostream>
#include <utility>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "mrw13.h"
#include "open_list_factory.h"
#include "random_walk_evaluator_factory.h"
#include "search_graph_factory.h"

namespace pplanner {

using std::array;
using std::pair;
using std::unordered_set;
using std::vector;

void MultiFocusGBFS::Init(const boost::property_tree::ptree &pt) {
  std::random_device seed_gen;
  engine_ = std::mt19937(seed_gen());
  //engine_ = std::mt19937(3694943095);

  if (auto uniform = pt.get_optional<int>("uniform"))
    uniform_ = uniform.get() == 1;

  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  bool keep_cost = false;
  if (auto opt = pt.get_optional<int>("keep_cost")) keep_cost = true;

  bool use_landmark = false;
  if (auto opt = pt.get_optional<int>("landmark")) use_landmark = true;

  bool dump_nodes = false;
  if (auto opt = pt.get_optional<int>("dump_nodes")) dump_nodes = true;

  graph_ = SearchGraphFactory(
      problem_, closed_exponent, keep_cost, use_landmark, dump_nodes);

  std::shared_ptr<Evaluator> friend_evaluator = nullptr;

  bool first = true;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type& child,
                 pt.get_child("evaluators")) {
    auto e = child.second;

    if (first) {
      rw_evaluator_ = RandomWalkEvaluatorFactory(problem_, e);
      first = false;
    }

    auto evaluator = EvaluatorFactory(problem_, graph_, friend_evaluator, e);
    evaluators_.push_back(evaluator);
    friend_evaluator = evaluator;
  }

  if (auto preferring = pt.get_child_optional("preferring"))
    use_preferred_ = true;

  open_list_option_ = pt.get_child("open_list");

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram"))
    ram = opt.get();

  graph_->ReserveByRAMSize(ram);
}

void MultiFocusGBFS::UpdateQ(const vector<int> &applicable,
                             const unordered_set<int> &preferred) {
  qw_max_ = 1.0;

  for (auto a : preferred) {
    q1_[a] = std::min(q1_[a] * e1_, 1.0e250);
    qw_[a] = std::min(qw_[a] * ew_, 1.0e250);

    if (qw_[a] > qw_max_) qw_max_ = qw_[a];
  }
}

int MultiFocusGBFS::RWEvaluate(const vector<int> &state,
                               const vector<int> &applicable,
                               unordered_set<int> &preferred) {
  if (uniform_) return rw_evaluator_->Evaluate(state);

  int h = rw_evaluator_->Evaluate(state, applicable, preferred);
  UpdateQ(applicable, preferred);

  return h;
}

void MultiFocusGBFS::RWInitialEvaluate() {
  rw_best_state_ = problem_->initial();
  generator_->Generate(rw_best_state_, rw_best_applicable_);
  rw_best_h_ = RWEvaluate(rw_best_state_, rw_best_applicable_,
                          rw_best_preferred_);
  rw_evaluator_->UpdateBest();
  ++generated_;
  ++rw_evaluated_;
  ++evaluated_;
  std::cout << "initial h value for MRW: " << rw_best_h_ << std::endl;
  rw_initial_h_ = rw_best_h_;
  rw_initial_applicable_ = rw_best_applicable_;
  rw_initial_preferred_ = rw_best_preferred_;
}

int MultiFocusGBFS::MHA(const vector<int> &applicable,
                        unordered_set<int> &preferred) {
  assert(!applicable.empty());
  double cumsum = 0.0;
  int best = applicable[0];

  for (auto a : applicable) {
    double score = 0.0;

    if (uniform_) {
      score = e1_;
    } else if (preferred.empty()) {
      score = q1_[a];
    } else if (preferred.find(a) != preferred.end()) {
      score = (q1_[a] / qw_[a]) * qw_max_;
    } else {
      score = q1_[a] / qw_[a];
    }

    cumsum += score;
    double p = dist_(engine_);

    if (p < score / cumsum) best = a;
  }

  return best;
}

void MultiFocusGBFS::AddNewFocus(int h, const vector<int> &state,
                                 const vector<int> &sequence) {
  thread_local vector<int> values;

  int node = graph_->GenerateNode(-1, -1, state);
  graph_->SetH(node, h);
  rw_evaluator_->CopyBestToSearchGraph(node, graph_);

  best_h_ = h;
  open_lists_.push_back(SharedOpenListFactory(open_list_option_,
                                              evaluators_));
  open_lists_.back()->Push(values, node, true);
  rw_plan_to_node_[node] = sequence;
  std::cout << "new focus h=" << h << std::endl;
}

void MultiFocusGBFS::UpdateBest(int h, const std::vector<int> &state,
                                const std::vector<int> &applicable,
                                const std::unordered_set<int> &preferred,
                                const std::vector<int> &sequence) {
  rw_best_h_ = h;
  rw_best_state_ = state;
  rw_best_applicable_ = applicable;
  rw_best_preferred_ = preferred;
  rw_plan_.insert(rw_plan_.end(), sequence.begin(), sequence.end());
  //std::cout << "New best heuristic value for MRW: " << rw_best_h_ << std::endl;

  if (h < best_h_) {
    AddNewFocus(h, state, rw_plan_);
    best_h_ = h;
  }
}

void MultiFocusGBFS::GlobalRestart(int li) {
  ++n_restarts_;

  if (li != 0) {
    double v_i_w = static_cast<double>(rw_initial_h_ - rw_best_h_);
    v_i_w /= static_cast<double>(li);
    v_w_ += (v_i_w - v_w_) / static_cast<double>(n_restarts_);
    tg_ = rw_initial_h_ / v_w_;
  }

  rw_best_state_ = problem_->initial();
  rw_best_applicable_ = rw_initial_applicable_;
  rw_best_preferred_ = rw_initial_preferred_;

  rw_evaluator_->GlobalRestart();
  rw_best_h_ = RWEvaluate(rw_best_state_, rw_best_applicable_,
                          rw_best_preferred_);
  ++rw_evaluated_;
  ++evaluated_;
  rw_plan_.clear();
  std::fill(q1_.begin(), q1_.end(), 1.0);
  std::fill(qw_.begin(), qw_.end(), 1.0);

  //std::cout << "restart new tg=" << tg_ << std::endl;
}

bool MultiFocusGBFS::Walk(int *w, int *li) {
  thread_local vector<int> state;
  thread_local vector<int> applicable;
  thread_local unordered_set<int> preferred;
  thread_local vector<int> sequence;

  state = rw_best_state_;
  applicable = rw_best_applicable_;
  preferred = rw_best_preferred_;
  sequence.clear();

  int before_evaluated = rw_evaluated_;
  int arg_rl = ChoiceRl(eps_, value_rls_, cost_rls_, engine_);
  double rl = GetRl<double>(arg_rl, rls_);
  int result_h = -1;

  while (true) {
    int a = MHA(applicable, preferred);
    ++expanded_;

    problem_->ApplyEffect(a, state);
    ++generated_;
    sequence.push_back(a);

    generator_->Generate(state, applicable);
    int h = RWEvaluate(state, applicable, preferred);
    ++rw_evaluated_;
    ++evaluated_;

    if (problem_->IsGoal(state)) {
      rw_plan_.insert(rw_plan_.end(), sequence.begin(), sequence.end());

      return true;
    }

    if (h == -1 || applicable.empty()) {
      ++dead_ends_;
      rw_evaluator_->LocalRestart();
      result_h = -1;
      break;
    }

    if (h < rw_best_h_) {
      rw_evaluator_->UpdateBest();
      result_h = h;
      break;
    }

    if (dist_(engine_) < rl) {
      rw_evaluator_->LocalRestart();
      result_h = -1;
      break;
    }
  }

  ++(*w);
  int cost = rw_evaluated_ - before_evaluated;

  if (result_h < rw_best_h_ && result_h != -1) {
    Feedback(arg_rl, rw_best_h_ - result_h, cost, value_rls_, cost_rls_);
    *li = *w;
    UpdateBest(result_h, state, applicable, preferred, sequence);
  } else {
    Feedback(arg_rl, 0, cost, value_rls_, cost_rls_);
  }

  if (*w - *li > tg_) {
    GlobalRestart(*li);
    *w = 0;
  }

  return false;
}

int MultiFocusGBFS::Evaluate(const vector<int> &state, int node,
                             vector<int> &values) {
  values.clear();

  for (auto e : evaluators_) {
    int h = e->Evaluate(state, node);
    values.push_back(h);

    if (h == -1) return -1;
  }

  return values[0];
}

void MultiFocusGBFS::InitialEvaluate() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);
  std::vector<int> values;
  best_h_ = Evaluate(state, node, values);
  graph_->SetH(node, best_h_);
  open_lists_.push_back(OpenListFactory(open_list_option_, evaluators_));
  open_lists_[0]->Push(values, node, false);
  std::cout << "Initial heuristic value for GBFS: " << best_h_ << std::endl;
}

std::shared_ptr<OpenList> MultiFocusGBFS::GreedyOpen() {
  thread_local std::vector<int> minimum_values;

  auto iter = std::remove_if(open_lists_.begin(), open_lists_.end(),
                             [](std::shared_ptr<OpenList> p)->bool
                             { return p->IsEmpty(); });
  open_lists_.erase(iter, open_lists_.end());
  std::shared_ptr<OpenList> result = nullptr;

  for (auto open : open_lists_) {
    if (result == nullptr || open->MinimumValues() < minimum_values) {
      result = open;
      minimum_values = open->MinimumValues();
    }
  }

  return result;
}

int MultiFocusGBFS::Expand() {
  thread_local vector<int> state(problem_->n_variables());
  thread_local vector<int> child(problem_->n_variables());
  thread_local vector<int> applicable;
  thread_local unordered_set<int> preferred;
  thread_local vector<int> values;

  auto open = GreedyOpen();

  if (open == nullptr) return -1;

  int node = open->Pop();

  if (!graph_->CloseIfNot(node)) return -1;

  graph_->Expand(node, state);
  ++expanded_;

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    ++dead_ends_;
    return -1;
  }

  if (use_preferred_)
    preferring_->Evaluate(state, node, applicable, preferred);

  for (auto o : applicable) {
    child = state;
    problem_->ApplyEffect(o, child);

    int child_node = graph_->GenerateNodeIfNotClosed(o, node, state, child);
    if (child_node == -1) continue;
    ++generated_;

    bool is_preferred = use_preferred_ && preferred.find(o) != preferred.end();
    int h = Evaluate(child, child_node, values);
    graph_->SetH(child_node, h);
    ++evaluated_;

    if (h == -1) {
      ++dead_ends_;
      continue;
    }

    open->Push(values, child_node, is_preferred);

    if (h < best_h_) {
      best_h_ = h;
      std::cout << "New best heuristic value for GBFS: "
                << best_h_ << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, "
                << expanded_ << " expanded]" << std::endl;

      if (use_preferred_) open->Boost();
    }
  }

  return -1;
}

vector<int> MultiFocusGBFS::Plan() {
  InitialEvaluate();
  RWInitialEvaluate();
  int w = 0;
  int li = 0;

  while (!open_lists_.empty()) {
    if (Walk(&w, &li)) return rw_plan_;

    int node = Expand();

    if (node != -1) return ExtractPlan(node);
  }

  return vector<int>{-1};
}

std::vector<int> MultiFocusGBFS::ExtractPlan(int node) {
  if (node == -1) return std::vector<int>{-1};

  vector<int> result;

  while (graph_->Parent(node) != -1
         || rw_plan_to_node_.find(node) != rw_plan_to_node_.end()) {
    if (graph_->Parent(node) == -1) {
      std::cout << "GBFS searched a state found by MRW" << std::endl;
      result.insert(result.begin(), rw_plan_to_node_[node].begin(),
                    rw_plan_to_node_[node].end());

      return result;
    } else {
      result.insert(result.begin(), graph_->Action(node));
    }

    node = graph_->Parent(node);
  }

  return result;
}
void MultiFocusGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
}

} // namespace pplanner
