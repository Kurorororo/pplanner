#include "mrw13.h"

#include <array>
#include <iostream>
#include <utility>

#include "evaluator_factory.h"

namespace pplanner {

using std::array;
using std::pair;
using std::unordered_set;
using std::vector;

void Feedback(int arg_rl, int value, int cost, vector<int> &value_rls,
              vector<int> &cost_rls) {
  value_rls[arg_rl] += value;
  cost_rls[arg_rl] += cost;
}

int RandomRl(size_t size, std::default_random_engine &engine) {
  return engine() % size;
}

int ChoiceRl(double eps, const vector<int> &value_rls,
             const vector<int> &cost_rls, std::default_random_engine &engine) {
  std::uniform_real_distribution<> dist(0.0, 1.0);
  if (dist(engine) < eps) return RandomRl(value_rls.size(), engine);

  double q_max = 0.0;
  int arg_best = -1;

  for (int i=0, n=cost_rls.size(); i<n; ++i) {
    if (cost_rls[i] == 0) continue;

    double q = static_cast<double>(value_rls[i]);
    q /= static_cast<double>(cost_rls[i]);

    if (q > q_max) {
      q_max = q;
      arg_best = i;
    }
  }

  if (arg_best == -1) return RandomRl(value_rls.size(), engine);

  return arg_best;
}

void Mrw13::Init(const boost::property_tree::ptree &pt) {
  std::random_device seed_gen;
  engine_ = std::default_random_engine(seed_gen());

  auto heuristic = pt.get_child_optional("heuristic");
  if (!heuristic) throw std::runtime_error("No heuristic is specified.");
  evaluator_ = EvaluatorFactory(problem_, heuristic.get());

  auto preferring = pt.get_child_optional("preferring");

  if (!preferring)
    same_ = true;
  else
    preferring_ = EvaluatorFactory(problem_, preferring.get());

  if (auto option = pt.get_child_optional("option")) {
    if (auto uniform = option->get_optional<int>("uniform"))
      uniform_ = uniform.get() == 1;

    if (auto fix = option->get_optional<int>("fix"))
      fix_ = fix.get() == 1;
  }

  if (auto option = pt.get_optional<int>("measure"))
    measure_ = option.get() == 1;

  if (measure_)
    is_preferred_operator_.resize(problem_->n_actions(), false);
}

vector<int> Mrw13::Plan() {
  auto best_state = problem_->initial();
  ++generated_;

  vector<int> best_applicable;
  generator_->Generate(best_state, best_applicable);

  unordered_set<int> best_preferred;
  int best_h = Evaluate(best_state, best_applicable, best_preferred);
  ++evaluated_;

  if (best_h == -1) return vector<int>{-1};
  std::cout << "initial h value: " << best_h << std::endl;

  if (problem_->IsGoal(best_state)) return plan_;

  int initial_h = best_h;
  auto initial_applicable = best_applicable;
  auto initial_preferred = best_preferred;

  int r = 0;
  int w = 0;
  int li = 0;
  double v_w = 0.0;
  int walks = 0;

  vector<int> state;
  vector<int> applicable;
  unordered_set<int> preferred;
  vector<int> sequence;

  while (true) {
    sequence.clear();
    state = best_state;
    applicable = best_applicable;
    preferred = best_preferred;

    int before_evaluated = evaluated_;
    int arg_rl = ChoiceRl(eps_, value_rls_, cost_rls_, engine_);
    int h;

    if (measure_) {
      tmp_is_preferred_successor_.clear();
      tmp_n_preferred_successors_.clear();
      tmp_n_successors_.clear();
    }

    if (fix_) {
      int length = GetRl<int>(arg_rl, ls_);
      h = Walk(best_h, length, state, sequence, applicable, preferred);
    } else {
      double rl = GetRl<double>(arg_rl, rls_);
      h = Walk(best_h, rl, state, sequence, applicable, preferred);
    }

    ++w;
    ++walks;

    if (problem_->IsGoal(state)) {
      plan_.insert(plan_.end(), sequence.begin(), sequence.end());

      if (measure_) {
        is_preferred_successor_.insert(is_preferred_successor_.end(),
                                       tmp_is_preferred_successor_.begin(),
                                       tmp_is_preferred_successor_.end());
        n_preferred_successors_.insert(n_preferred_successors_.end(),
                                       tmp_n_preferred_successors_.begin(),
                                       tmp_n_preferred_successors_.end());
        n_successors_.insert(n_successors_.end(), tmp_n_successors_.begin(),
                             tmp_n_successors_.end());
      }

      if (measure_) ActionElimination();

      return plan_;
    }

    int cost = evaluated_ - before_evaluated;

    if (h < best_h && h != -1) {
      Feedback(arg_rl, best_h- h, cost, value_rls_, cost_rls_);

      best_h = h;
      best_state = state;
      best_applicable = applicable;
      best_preferred = preferred;
      plan_.insert(plan_.end(), sequence.begin(), sequence.end());
      li = w;

      if (measure_) {
        is_preferred_successor_.insert(is_preferred_successor_.end(),
                                       tmp_is_preferred_successor_.begin(),
                                       tmp_is_preferred_successor_.end());
        n_preferred_successors_.insert(n_preferred_successors_.end(),
                                       tmp_n_preferred_successors_.begin(),
                                       tmp_n_preferred_successors_.end());
        n_successors_.insert(n_successors_.end(), tmp_n_successors_.begin(),
                             tmp_n_successors_.end());
      }

      //std::cout << "New best heuristic value : " << h
      //          << std::endl;
      //std::cout << "#walks " << walks << std::endl;
      //std::cout << "[" << evaluated_
      //          << " evaluated, " << expanded_ << " expanded]" << std::endl;
    } else {
      Feedback(arg_rl, 0, cost, value_rls_, cost_rls_);
    }

    if (w - li > tg_) {
      ++r;
      double v_i_w = static_cast<double>(initial_h - best_h);
      v_i_w /= static_cast<double>(li);
      v_w += (v_i_w - v_w) / static_cast<double>(r);
      tg_ = initial_h / v_w;
      w = 0;

      best_state = problem_->initial();
      best_applicable = initial_applicable;
      best_preferred = initial_preferred;

      best_h = Evaluate(best_state, best_applicable, best_preferred);
      plan_.clear();

      if (measure_) {
        is_preferred_successor_.clear();
        n_preferred_successors_.clear();
      }

      std::fill(q1_.begin(), q1_.end(), 1.0);
      std::fill(qw_.begin(), qw_.end(), 1.0);

      //std::cout << "restart new tg=" << tg_ << std::endl;
    }
  }
}

void Mrw13::UpdateQ(const vector<int> &applicable,
                    const unordered_set<int> &preferred) {
  qw_max_ = 1.0;

  for (auto a : preferred) {
    q1_[a] = std::min(q1_[a] * e1_, 1.0e250);
    qw_[a] = std::min(qw_[a] * ew_, 1.0e250);

    if (qw_[a] > qw_max_) qw_max_ = qw_[a];

    if (measure_) is_preferred_operator_[a] = true;
  }
}

int Mrw13::Evaluate(const vector<int> &state, const vector<int> &applicable,
                    unordered_set<int> &preferred) {
  if (uniform_) return evaluator_->Evaluate(state, -1);

  if (same_) {
    int h = evaluator_->Evaluate(state, -1, applicable, preferred);
    n_preferreds_ += preferred.size();
    UpdateQ(applicable, preferred);

    return h;
  }

  preferring_->Evaluate(state, -1, applicable, preferred);
  n_preferreds_ += preferred.size();
  UpdateQ(applicable, preferred);

  return evaluator_->Evaluate(state, -1);
}

int Mrw13::Walk(int best_h, int length, vector<int> &state,
                vector<int> &sequence, vector<int> &applicable,
                unordered_set<int> &preferred) {
  auto current_state = state;
  auto current_applicable = applicable;
  auto current_preferred = preferred;
  int path_length = 0;

  for (int i=0; i<length; ++i) {
    int a = MHA(current_applicable, current_preferred);
    ++expanded_;

    problem_->ApplyEffect(a, current_state);
    ++generated_;

    sequence.push_back(a);

    generator_->Generate(current_state, current_applicable);
    int h = Evaluate(current_state, current_applicable, current_preferred);
    ++evaluated_;

    if (h == -1 || current_applicable.empty()) {
      sequence.resize(path_length);
      ++dead_ends_;

    if (measure_) {
      tmp_is_preferred_successor_.resize(path_length);
      tmp_n_preferred_successors_.resize(path_length);
      tmp_n_successors_.resize(path_length);
    }

      return best_h;
    }

    if (problem_->IsGoal(current_state)) {
      path_length = i + 1;
      state = current_state;
      applicable = current_applicable;
      preferred = current_preferred;

      return 0;
    }

    if (h < best_h) {
      path_length = i + 1;
      best_h = h;
      state = current_state;
      applicable = current_applicable;
      preferred = current_preferred;
    }
  }

  sequence.resize(path_length);

  if (measure_) {
    tmp_is_preferred_successor_.resize(path_length);
    tmp_n_preferred_successors_.resize(path_length);
    tmp_n_successors_.resize(path_length);
  }

  return best_h;
}

int Mrw13::Walk(int best_h, double rl, vector<int> &state, vector<int> &sequence,
                vector<int> &applicable, unordered_set<int> &preferred) {
  while (true) {
    int a = MHA(applicable, preferred);
    ++expanded_;

    problem_->ApplyEffect(a, state);
    ++generated_;

    sequence.push_back(a);

    generator_->Generate(state, applicable);
    int h = Evaluate(state, applicable, preferred);
    ++evaluated_;

    if (h == -1 || applicable.empty()) {
      ++dead_ends_;

      return h;
    }

    if (problem_->IsGoal(state)) return 0;

    if (h < best_h) return h;

    if (dist_(engine_) < rl) return -1;
  }
}

int Mrw13::MHA(const vector<int> &applicable, unordered_set<int> &preferred) {
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

  if (measure_) {
    if (preferred.find(best) != preferred.end())
      tmp_is_preferred_successor_.push_back(true);
    else
      tmp_is_preferred_successor_.push_back(false);

    tmp_n_preferred_successors_.push_back(preferred.size());
    tmp_n_successors_.push_back(applicable.size());
  }

  return best;
}

void Mrw13::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
  std::cout << "Preferred operators " << n_preferreds_ << std::endl;
  double p_p_e = static_cast<double>(n_preferreds_)
    / static_cast<double>(evaluated_);
  std::cout << "Preferreds per state " << p_p_e << std::endl;

  if (measure_) DumpPreferringMetrics();
}

void Mrw13::ActionElimination() {
  auto state = problem_->initial();

  int i = 0;
  int n = plan_.size();
  vector<bool> marked(plan_.size(), false);

  while (i < n) {
    marked[i] = true;
    auto successor = state;
    vector<pair<int, int> > precondition;

    for (int j=i+1; j<n; ++j) {
      problem_->CopyPrecondition(plan_[j], precondition);

      for (auto p : precondition) {
        if (successor[p.first] != p.second) {
          marked[j] = true;
          break;
        }
      }

      if (!marked[j]) problem_->ApplyEffect(plan_[j], successor);
    }

    if (problem_->IsGoal(successor)) {
      vector<int> new_plan;
      vector<bool> new_is_preferred_successor_;
      vector<int> new_n_preferred_successors_;
      vector<int> new_n_successors_;

      for (int j=0; j<n; ++j) {
        if (marked[j]) continue;

        new_plan.push_back(plan_[j]);
        new_is_preferred_successor_.push_back(is_preferred_successor_[j]);
        new_n_preferred_successors_.push_back(n_preferred_successors_[j]);
        new_n_successors_.push_back(n_successors_[j]);
      }

      plan_.swap(new_plan);
      is_preferred_successor_.swap(new_is_preferred_successor_);
      n_preferred_successors_.swap(new_n_preferred_successors_);
      n_successors_.swap(new_n_successors_);
      marked.resize(plan_.size());
      n = plan_.size();
      continue;
    } else {
      problem_->ApplyEffect(plan_[i], state);
      ++i;
    }

    std::fill(marked.begin(), marked.end(), false);
  }
}

void Mrw13::DumpPreferringMetrics() const {
  vector<bool> in_plan(problem_->n_actions(), false);

  for (auto a : plan_)
    in_plan[a] = true;

  int op_tp = 0;
  int op_fn = 0;
  int op_fp = 0;
  int op_tn = 0;

  for (int i=0, n=problem_->n_actions(); i<n; ++i) {
    if (in_plan[i] && is_preferred_operator_[i])
      ++op_tp;

    if (in_plan[i] && !is_preferred_operator_[i])
      ++op_fn;

    if (!in_plan[i] && is_preferred_operator_[i])
      ++op_fp;

    if (!in_plan[i] && !is_preferred_operator_[i])
      ++op_tn;
  }

  double op_ac = static_cast<double>(op_tp + op_tn)
    / static_cast<double>(op_tp + op_fn + op_fp + op_tn);
  double op_pr = static_cast<double>(op_tp) / static_cast<double>(op_tp + op_fp);
  double op_re = static_cast<double>(op_tp) / static_cast<double>(op_tp + op_fn);
  double op_f = (2.0 * op_re * op_pr) / (op_re + op_pr);

  int st_tp = 0;
  int st_fn = 0;
  int st_fp = 0;
  int st_tn = 0;

  for (int i=0, n=plan_.size(); i<n; ++i) {
    if (is_preferred_successor_[i]) {
      ++st_tp;
      st_fp += n_preferred_successors_[i] - 1;
      st_tn += n_successors_[i] - n_preferred_successors_[i];
    } else {
      ++st_fn;
      st_fp += n_preferred_successors_[i];
      st_tn += n_successors_[i] - n_preferred_successors_[i] - 1;
    }
  }

  double st_ac = static_cast<double>(st_tp + st_tn)
    / static_cast<double>(st_tp + st_fn + st_fp + st_tn);
  double st_pr = static_cast<double>(st_tp) / static_cast<double>(st_tp + st_fp);
  double st_re = static_cast<double>(st_tp) / static_cast<double>(st_tp + st_fn);
  double st_f = (2.0 * st_re * st_pr) / (st_re + st_pr);

  std::cout << std::endl;
  std::cout << "Operator TP " << op_tp << std::endl;
  std::cout << "Operator FN " << op_fn << std::endl;
  std::cout << "Operator FP " << op_fp << std::endl;
  std::cout << "Operator TN " << op_tn << std::endl;
  std::cout << "Operator Accuracy " << op_ac << std::endl;
  std::cout << "Operator Precision " << op_pr << std::endl;
  std::cout << "Operator Recall " << op_re << std::endl;
  std::cout << "Operator F " << op_f << std::endl;

  std::cout << std::endl;
  std::cout << "State TP " << st_tp << std::endl;
  std::cout << "State FN " << st_fn << std::endl;
  std::cout << "State FP " << st_fp << std::endl;
  std::cout << "State TN " << st_tn << std::endl;
  std::cout << "State Accuracy " << st_ac << std::endl;
  std::cout << "State Precision " << st_pr << std::endl;
  std::cout << "State Recall " << st_re << std::endl;
  std::cout << "State F " << st_f << std::endl;
  std::cout << std::endl;
}

} // namespace pplanner
