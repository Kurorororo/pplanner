#include "../mrw13.h"

#include <array>
#include <iostream>
#include <limits>
#include <unordered_set>

using std::array;
using std::unordered_set;
using std::vector;

namespace rwls {

template<class H>
int Mrw13<H>::MHA(const vector<int> &applicable,
                  unordered_set<int> &preferred) {
  std::uniform_real_distribution<> dist(0.0, 1.0);

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
    double p = dist(engine_);

    if (p < score / cumsum) best = a;
  }

  return best;
}

template<class H>
void Mrw13<H>::UpdateQ(const vector<int> &applicable,
                       const unordered_set<int> &preferred) {
  qw_max_ = 1.0;

  for (auto a : preferred) {
    q1_[a] = std::min(q1_[a] * e1_, 1.0e250);
    qw_[a] = std::min(qw_[a] * ew_, 1.0e250);

    if (qw_[a] > qw_max_) qw_max_ = qw_[a];
  }

}

template<class H>
int Mrw13<H>::Walk(int h_min, int length, State &state, vector<int> &sequence,
                   unordered_set<int> &preferred, vector<int> &applicable) {
  std::uniform_real_distribution<> dist(0.0, 1.0);
  State current = state;
  auto current_applicable = applicable;
  int path_length = 0;

  for (int i=0; i<length; ++i) {
    int a = MHA(current_applicable, preferred);
    ++expanded;

    ApplyEffect(domain_.effects[a], current);
    ++generated;

    sequence.push_back(a);

    FindFromTable(table_, domain_, current, current_applicable);
    int h = heuristic_(current, domain_, current_applicable, preferred);
    ++evaluated;
    ++evaluations;

    if (h == std::numeric_limits<int>::max()) {
      sequence.resize(path_length);
      ++deadend;

      return h_min;
    }

    if (!uniform_) UpdateQ(applicable, preferred);

    if (GoalCheck(domain_.goal, current)) {
      path_length = i + 1;
      state = current;
      applicable = current_applicable;

      return 0;
    }

    if (h < h_min) {
      path_length = i + 1;
      h_min = h;
      state = current;
      applicable = current_applicable;
    }
  }

  sequence.resize(path_length);

  return h_min;
}

template<class H>
int Mrw13<H>::Walk(int h_min, double rl, State &state, vector<int> &sequence,
                   unordered_set<int> &preferred, vector<int> &applicable) {
  std::uniform_real_distribution<> dist(0.0, 1.0);

  while (true) {
    int a = MHA(applicable, preferred);
    ++expanded;

    ApplyEffect(domain_.effects[a], state);
    ++generated;

    sequence.push_back(a);

    FindFromTable(table_, domain_, state, applicable);
    int h = heuristic_(state, domain_, applicable, preferred);
    ++evaluated;
    ++evaluations;

    if (h == std::numeric_limits<int>::max()) {
      ++deadend;
      return h;
    }

    if (!uniform_) UpdateQ(applicable, preferred);

    if (GoalCheck(domain_.goal, state)) return 0;

    if (h < h_min) return h;

    if (dist(engine_) < rl) return std::numeric_limits<int>::max();
  }
}

template<class H>
vector<int> Mrw13<H>::operator()(bool fix) {
  unordered_set<int> preferred;

  vector<int> sequence;
  vector<int> plan;
  vector<int> applicable;

  ++generated;

  FindFromTable(table_, domain_, domain_.initial, applicable);
  int initial_h = heuristic_(domain_.initial, domain_, applicable, preferred);
  ++evaluated;
  ++evaluations;

  if (initial_h == std::numeric_limits<int>::max()) return vector<int>{-1};
  std::cout << "initial h value: " << initial_h << std::endl;

  if (!uniform_) UpdateQ(applicable, preferred);

  if (GoalCheck(domain_.goal, domain_.initial)) return plan;

  int h_min = initial_h;
  auto current = domain_.initial;
  auto state = domain_.initial;
  auto current_applicable = applicable;
  auto initial_applicable = applicable;

  auto best_preferred = preferred;
  auto initial_preferred = preferred;

  std::uniform_real_distribution<> dist(0.0, 1.0);

  int r = 0;
  int w = 0;
  int li = 0;
  double v_w = 0.0;
  int walks = 0;

  while (true) {
    sequence.clear();
    state = current;
    applicable = current_applicable;
    preferred = best_preferred;

    int before_evaluated = evaluated;
    int arg_rl = ChoiceRl(eps_, value_rls_, cost_rls_, engine_);
    int h;

    if (fix) {
      int length = GetRl<int>(arg_rl, ls_);
      h = Walk(h_min, length, state, sequence, preferred, applicable);
    } else {
      double rl = GetRl<double>(arg_rl, rls_);
      h = Walk(h_min, rl, state, sequence, preferred, applicable);
    }

    ++w;
    ++walks;

    if (GoalCheck(domain_.goal, state)) {
      plan.insert(plan.end(), sequence.begin(), sequence.end());

      return plan;
    }

    int cost = evaluated - before_evaluated;

    if (h < h_min) {
      Feedback(arg_rl, h_min - h, cost, value_rls_, cost_rls_);

      h_min = h;
      current = state;
      current_applicable = applicable;
      best_preferred = preferred;
      plan.insert(plan.end(), sequence.begin(), sequence.end());
      li = w;

      //std::cout << "New best heuristic value for ff: " << h
      //          << std::endl;
      //std::cout << "#walks " << walks << std::endl;
      //std::cout << "[" << evaluated
      //          << " evaluated, " << expanded << " expanded]" << std::endl;


    } else {
      Feedback(arg_rl, 0, cost, value_rls_, cost_rls_);
    }

    if (w - li > tg_) {
      ++r;
      double v_i_w = static_cast<double>(initial_h - h_min);
      v_i_w /= static_cast<double>(li);
      v_w += (v_i_w - v_w) / static_cast<double>(r);
      tg_ = initial_h / v_w;
      w = 0;

      h_min = initial_h;
      current = domain_.initial;
      current_applicable = initial_applicable;
      plan.clear();

      std::fill(q1_.begin(), q1_.end(), 1.0);
      std::fill(qw_.begin(), qw_.end(), 1.0);

      best_preferred = initial_preferred;
      UpdateQ(current_applicable, best_preferred);

      //std::cout << "restart new tg=" << tg_ << std::endl;
    }
  }
}

} // namespace rwls
