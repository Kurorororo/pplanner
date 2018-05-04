#include "mrw13.h"

#include <array>
#include <iostream>

#include "evaluator_factory.h"

namespace pplanner {

using std::array;
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

  auto preferring = pt.get_child_optional("preferring");

  if (!preferring) {
    same_ = true;
    preferring = heuristic;
  }

  if (auto options = pt.get_child_optional("options")) {
    if (auto uniform = options->get_optional<int>("uniform")) {
      uniform_ = uniform.get() == 1;
    }

    if (auto fix = options->get_optional<int>("fix")) {
      fix_ = fix.get() == 1;
    }
  }

  evaluator_ = EvaluatorFactory(problem_, heuristic.get());

  if (!same_) preferring_ = EvaluatorFactory(problem_, preferring.get());
}

vector<int> Mrw13::Plan() {
  auto best_state = problem_->initial();
  ++generated_;

  vector<int> best_applicable;
  generator_->Generate(best_state, best_applicable);

  unordered_set<int> best_preferred;
  int best_h = Evaluate(best_state, best_applicable, best_preferred);
  ++evaluated_;

  if (best_h== -1) return vector<int>{-1};
  std::cout << "initial h value: " << best_h << std::endl;

  vector<int> plan;
  if (problem_->IsGoal(best_state)) return plan;

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
      plan.insert(plan.end(), sequence.begin(), sequence.end());

      return plan;
    }

    int cost = evaluated_ - before_evaluated;

    if (h < best_h && h != -1) {
      Feedback(arg_rl, best_h- h, cost, value_rls_, cost_rls_);

      best_h = h;
      best_state = state;
      best_applicable = applicable;
      best_preferred = preferred;
      plan.insert(plan.end(), sequence.begin(), sequence.end());
      li = w;

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
      plan.clear();

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

int Mrw13::MHA(const vector<int> &applicable, unordered_set<int> &preferred)
  const {
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

void Mrw13::DumpStatistics() const {
  std::cout << "Expanded " << expanded_ << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated_ << " state(s)" << std::endl;
  std::cout << "Generated " << generated_ << " state(s)" << std::endl;
  std::cout << "Dead ends " << dead_ends_ << " state(s)" << std::endl;
  std::cout << "Preferred operators  " << n_preferreds_ << " state(s)"
            << std::endl;
}

} // namespace pplanner
