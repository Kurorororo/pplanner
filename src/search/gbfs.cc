#include "search/gbfs.h"

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

void GBFS::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto opt = pt.get_optional<int>("exhaust")) exhaust_ = true;

  if (auto opt = pt.get_optional<int>("max_expansion")) {
    limit_expansion_ = true;
    max_expansion_ = opt.get();
  }

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  bool keep_cost = false;
  if (auto opt = pt.get_optional<int>("keep_cost")) keep_cost = true;

  bool use_landmark = false;
  if (auto opt = pt.get_optional<int>("landmark")) use_landmark = true;

  bool dump_nodes = false;
  if (auto opt = pt.get_optional<bool>("dump_nodes")) dump_nodes = opt.get();

  graph_ = SearchGraphFactory(problem_, closed_exponent, keep_cost,
                              use_landmark, dump_nodes);

  std::shared_ptr<Evaluator> friend_evaluator = nullptr;

  BOOST_FOREACH (const boost::property_tree::ptree::value_type &child,
                 pt.get_child("evaluators")) {
    auto e = child.second;
    auto evaluator = EvaluatorFactory(problem_, e, friend_evaluator, graph_);
    evaluators_.push_back(evaluator);
    friend_evaluator = evaluator;
  }

  if (auto preferring = pt.get_child_optional("preferring")) {
    use_preferred_ = true;

    if (auto name = preferring.get().get_optional<std::string>("name")) {
      if (name.get() == "same") {
        preferring_ = evaluators_[0];
      } else {
        preferring_ =
            EvaluatorFactory(problem_, preferring.get(), nullptr, graph_);
      }
    }
  }

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option);

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram")) ram = opt.get();

  if (auto opt = pt.get_optional<int>("dominance")) {
    use_dominance_ = true;
    lds_ = std::unique_ptr<LDS>(new LDS(problem_));
    ram -= lds_->n_bytes();
  }

  graph_->ReserveByRAMSize(ram);

  if (auto opt = pt.get_child_optional("sss")) {
    use_sss_ = true;
    sss_aproximater_ =
        std::unique_ptr<SSSApproximater>(new SSSApproximater(problem_));

    if (auto n_disable = opt.get().get_optional<int>("n_pruning_disable"))
      n_pruning_disable_ = n_disable.get();

    if (auto min_ratio = opt.get().get_optional<double>("min_pruning_ratio"))
      min_pruning_ratio_ = min_ratio.get();
  }
}

vector<int> GBFS::InitialExpand() {
  auto state = problem_->initial();
  int node = graph_->GenerateNode(-1, -1, state);
  ++generated_;

  std::vector<int> values;
  best_h_ = Evaluate(evaluators_, state, node, values);
  graph_->SetH(node, best_h_);
  open_list_->Push(values, node, true);
  std::cout << "Initial heuristic value: " << best_h_ << std::endl;
  ++evaluated_;

  return state;
}

int GBFS::NodeToExpand() {
  while (!NoNode()) {
    int node = open_list_->Pop();

    if (graph_->CloseIfNot(node)) return node;
  }

  return -1;
}

int GBFS::Expand(int node, vector<int> &state, vector<int> &child,
                 vector<int> &applicable, unordered_set<int> &preferred) {
  static vector<int> values;
  static vector<bool> sss;

  ++expanded_;
  graph_->Expand(node, state);

  if (problem_->IsGoal(state)) return node;

  if (use_sss_ && !sss_checked_ && expanded_ > n_pruning_disable_) {
    double ratio =
        static_cast<double>(n_pruned_) / static_cast<double>(n_branching_);

    std::cout << "#pruned=" << n_pruned_ << std::endl;
    std::cout << "#branching=" << n_branching_ << std::endl;
    std::cout << "ratio=" << ratio << std::endl;

    if (n_pruned_ == 0) use_sss_ = false;
    // if (ratio < min_pruning_ratio_) use_sss_ = false;

    sss_checked_ = true;
  }

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    ++dead_ends_;
    return -1;
  }

  if (use_preferred_) preferring_->Evaluate(state, node, applicable, preferred);

  ++n_preferred_evaluated_;
  n_branching_ += applicable.size();

  if (use_sss_) sss_aproximater_->ApproximateSSS(state, applicable, sss);

  for (auto o : applicable) {
    if (use_sss_ && !sss[o]) {
      ++n_pruned_;
      continue;
    }

    problem_->ApplyEffect(o, state, child);

    if (use_dominance_ && (lds_->Dominance(child, state) ||
                           lds_->Dominance(child, problem_->initial()))) {
      ++n_d_pruned_;
      continue;
    }

    int child_node = graph_->GenerateNodeIfNotClosed(o, node, state, child);
    if (child_node == -1) continue;
    ++generated_;

    int h = Evaluate(evaluators_, child, child_node, values);
    graph_->SetH(child_node, h);
    ++evaluated_;

    if (h == -1) {
      ++dead_ends_;
      continue;
    }

    bool is_preferred = use_preferred_ && preferred.find(o) != preferred.end();
    if (is_preferred) ++n_preferreds_;
    open_list_->Push(values, child_node, is_preferred);

    if (h < best_h_) {
      best_h_ = h;
      std::cout << "New best heuristic value: " << best_h_ << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, " << expanded_
                << " expanded]" << std::endl;

      if (use_preferred_) open_list_->Boost();
    }
  }

  return -1;
}

int GBFS::Search() {
  auto state = InitialExpand();
  vector<int> child(state);
  vector<int> applicable;
  unordered_set<int> preferred;
  int last_goal = -1;

  while (true) {
    int node = NodeToExpand();

    if (node == -1) return last_goal;

    int goal = Expand(node, state, child, applicable, preferred);

    if (limit_expansion_ && expanded_ > max_expansion_) return last_goal;

    if (goal != -1 && !exhaust_)
      return goal;
    else if (goal != -1)
      last_goal = goal;
  }

  return last_goal;
}

void GBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
  std::cout << "Preferred evaluated " << n_preferred_evaluated_ << " state(s)"
            << std::endl;
  std::cout << "Preferred successors " << n_preferreds_ << " state(s)"
            << std::endl;
  double p_p_e = static_cast<double>(n_preferreds_) /
                 static_cast<double>(n_preferred_evaluated_);
  std::cout << "Preferreds per state " << p_p_e << std::endl;
  double b_f = static_cast<double>(n_branching_) /
               static_cast<double>(n_preferred_evaluated_);
  std::cout << "Average branching factor " << b_f << std::endl;
  double p_p_b =
      static_cast<double>(n_preferreds_) / static_cast<double>(n_branching_);
  std::cout << "Preferred ratio " << p_p_b << std::endl;

  if (n_plan_step_ != -1 && expanded_ > 1) {
    double plan_length_per_expansion =
        static_cast<double>(n_plan_step_) / static_cast<double>(expanded_ - 1);
    std::cout << "Plan steps ratio " << plan_length_per_expansion << std::endl;
  } else {
    std::cout << "Plan steps ratio " << 0 << std::endl;
  }

  if (use_dominance_)
    std::cout << "Pruned by dominance " << n_d_pruned_ << std::endl;

  graph_->Dump();
}

}  // namespace pplanner
