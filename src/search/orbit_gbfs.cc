#include "search/orbit_gbfs.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include <boost/foreach.hpp>

#include "evaluator_factory.h"
#include "open_list_factory.h"
#include "search_graph_factory.h"

namespace pplanner {

using std::make_shared;
using std::pair;
using std::size_t;
using std::unordered_set;
using std::vector;

void OrbitGBFS::Init(const boost::property_tree::ptree &pt) {
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
  open_list_ = OpenListFactory(open_list_option);

  if (auto ram = pt.get_optional<size_t>("ram"))
    graph_->ReserveByRAMSize(ram.get());
  else
    graph_->ReserveByRAMSize(5000000000);

  //manager_->Dump();

  if (auto opt = pt.get_optional<int>("sss")) {
    use_sss_ = true;
    sss_aproximater_ = std::unique_ptr<SSSApproximater>(
        new SSSApproximater(problem_));
  }
}

vector<int> OrbitGBFS::InitialExpand() {
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

int OrbitGBFS::Expand(int node, vector<int> &state, vector<int> &child,
                 vector<int> &applicable, unordered_set<int> &preferred) {
  static std::vector<int> canonical(state);
  static std::vector<int> values;
  static vector<bool> sss;

  if (!graph_->CloseIfNot(node)) return -1;

  ++expanded_;
  graph_->Expand(node, state);

  if (problem_->IsGoal(state)) return node;

  generator_->Generate(state, applicable);

  if (applicable.empty()) {
    ++dead_ends_;
    return -1;
  }

  if (use_preferred_)
    preferring_->Evaluate(state, node, applicable, preferred);

  ++n_preferred_evaluated_;
  n_branching_ += applicable.size();

  if (use_sss_)
    sss_aproximater_->ApproximateSSS(state, applicable, sss);

  for (auto o : applicable) {
    if (use_sss_ && !sss[o]) continue;

    problem_->ApplyEffect(o, state, child);

    manager_->ToCanonical(child, canonical);

    int child_node = graph_->GenerateNodeIfNotClosed(o, node, canonical);
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
      std::cout << "[" << evaluated_ << " evaluated, "
                << expanded_ << " expanded]" << std::endl;

      if (use_preferred_) open_list_->Boost();
    }
  }

  return -1;
}

int OrbitGBFS::Search() {
  auto state = InitialExpand();
  vector<int> child(state);
  vector<int> applicable;
  unordered_set<int> preferred;
  int last_goal = -1;

  while (!NoNode()) {
    int node = NodeToExpand();
    int goal = Expand(node, state, child, applicable, preferred);

    if (limit_expansion_ && expanded_ > max_expansion_) return last_goal;

    if (goal != -1 && !exhaust_)
      return goal;
    else if (goal != -1)
      last_goal = goal;
  }

  return last_goal;
}

vector<pair<int, int> > OrbitGBFS::Trace(int node) const {
  vector<pair<int, int> > result;

  while (node != -1) {
    result.push_back(std::make_pair(graph_->Action(node), node));
    node = graph_->Parent(node);
  }

  std::reverse(result.begin(), result.end());

  return result;
}

vector<int> OrbitGBFS::TraceForward(const vector<pair<int, int> > &trace)
  const {
  vector<int> s_0(problem_->n_variables());
  vector<int> s_0_o(problem_->n_variables());
  vector<int> s_1(problem_->n_variables());
  vector<int> s_p_0(problem_->n_variables());
  vector<int> s_p_1(problem_->n_variables());
  vector<int> s_p_0_o(problem_->n_variables());
  vector<int> tmp_s(problem_->n_variables());
  vector<int> sigma;
  vector<int> sigma_i;
  vector<int> applicable;
  vector<int> plan;

  graph_->State(trace.begin()->second, s_p_0);
  s_0 = s_p_0;

  for (auto itr = trace.begin() + 1; itr != trace.end(); ++itr) {
    problem_->ApplyEffect(itr->first, s_p_0, s_p_0_o);
    manager_->ToCanonical(s_p_0_o, tmp_s, sigma_i);
    graph_->State(itr->second, s_p_1);

    //if (s_p_1 != tmp_s) {
    //  std::cout << "something is wrong!" << std::endl;
    //}

    std::reverse(sigma_i.begin(), sigma_i.end());
    sigma_i.insert(sigma_i.end(), sigma.begin(), sigma.end());
    sigma.swap(sigma_i);

    s_1 = s_p_1;
    tmp_s = s_p_1;

    for (int i=0, n=sigma.size(); i<n; ++i) {
      manager_->InversePermutate(sigma[i], tmp_s, s_1);
      if (i < (n - 1)) tmp_s.swap(s_1);
    }

    generator_->Generate(s_0, applicable);
    int min_cost = -1;
    int o_i = -1;

    for (auto o : applicable) {
      int cost = problem_->ActionCost(o);
      if (min_cost != -1 && cost >= min_cost) continue;

      problem_->ApplyEffect(o, s_0, s_0_o);

      if (s_0_o == s_1) {
        min_cost = cost;
        o_i = o;
      }
    }

    if (o_i == -1) return vector<int>{-1};

    plan.push_back(o_i);

    s_0.swap(s_1);
    s_p_0.swap(s_p_1);
  }

  return plan;
}

void OrbitGBFS::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
  std::cout << "Preferred evaluated " << n_preferred_evaluated_ << " state(s)"
            << std::endl;
  std::cout << "Preferred successors " << n_preferreds_ << " state(s)"
            << std::endl;
  double p_p_e = static_cast<double>(n_preferreds_)
    / static_cast<double>(n_preferred_evaluated_);
  std::cout << "Preferreds per state " << p_p_e << std::endl;
  double b_f = static_cast<double>(n_branching_)
    / static_cast<double>(n_preferred_evaluated_);
  std::cout << "Average branching factor " << b_f << std::endl;
  double p_p_b = static_cast<double>(n_preferreds_)
    / static_cast<double>(n_branching_);
  std::cout << "Preferred ratio " << p_p_b  << std::endl;

  graph_->Dump();
}

} // namespace pplanner
