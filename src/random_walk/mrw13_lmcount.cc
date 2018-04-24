#include "mrw13_lmcount.h"

#include <array>
#include <iostream>
#include <limits>

using std::array;
using std::vector;

namespace rwls {

int Mrw13Lmcount::Walk(int h_min, int length, State &state,
                       array<vector<bool>, 2> &accepted,
                       vector<bool> &best_accepted, vector<int> &sequence) {
  int c = 1;
  int path_length = 0;

  State current = state;

  for (int i=0; i<length; ++i) {
    int a = SampleFromTable(table_, domain_, current);
    ++expanded;

    if (a == -1) {
      ++deadend;
      sequence.resize(path_length);
      return h_min;
    }

    ApplyEffect(domain_.effects[a], current);
    ++generated;

    int h = lmcount_(current , domain_, accepted[1 - c], accepted[c]);
    ++evaluated;
    ++evaluations;

    if (h == std::numeric_limits<int>::max()) {
      ++deadend;
      sequence.resize(path_length);
      return h_min;
    }

    sequence.push_back(a);

    if (h < h_min) {
      path_length = i + 1;
      h_min = h;
      state = current;
      best_accepted = accepted[c];

      if (h == 0) return h_min;
    }

    c = 1 - c;
  }

  sequence.resize(path_length);

  return h_min;
}

int Mrw13Lmcount::Walk(int h_min, double rl, State &state,
                       array<vector<bool>, 2> &accepted,
                       vector<bool> &best_accepted, vector<int> &sequence) {
  std::uniform_real_distribution<> dist(0.0, 1.0);
  int c = 1;

  while (true) {
    int a = SampleFromTable(table_, domain_, state);
    ++expanded;

    if (a == -1) {
      ++deadend;
      return std::numeric_limits<int>::max();
    }

    ApplyEffect(domain_.effects[a], state);
    ++generated;

    int h = lmcount_(state, domain_, accepted[1 - c], accepted[c]);
    ++evaluated;
    ++evaluations;

    if (h == std::numeric_limits<int>::max()) {
      ++deadend;
      return h;
    }

    sequence.push_back(a);

    if (h < h_min) {
      best_accepted = accepted[c];
      return h;
    }

    if (dist(engine_) < rl) return std::numeric_limits<int>::max();

    c = 1 - c;
  }
}

vector<int> Mrw13Lmcount::operator()(bool fix) {
  array<vector<bool>, 2> accepted;
  accepted[0].resize(lmcount_.landmark_id_max);
  accepted[1].resize(lmcount_.landmark_id_max);
  vector<bool> best_accepted(lmcount_.landmark_id_max);

  vector<int> state(domain_.initial.size());
  vector<int> current(domain_.initial.size());
  vector<int> sequence;
  vector<int> plan;

  ++generated;

  int initial_h = lmcount_(domain_.initial, domain_, accepted[0], accepted[1],
                           true);
  ++evaluated;
  ++evaluations;

  if (initial_h == std::numeric_limits<int>::max()) return vector<int>{-1};
  std::cout << "initial h value: " << initial_h << std::endl;

  if (initial_h == 0) return plan;

  int h_min = initial_h;
  vector<bool> initial_accepted = accepted[1];

  current = domain_.initial;
  best_accepted = initial_accepted;

  std::uniform_real_distribution<> dist(0.0, 1.0);

  int r = 0;
  int w = 0;
  int li = 0;
  double v_w = 0.0;

  while (true) {
    sequence.clear();
    accepted[0] = best_accepted;
    state = current;

    int before_evaluated = evaluated;
    int arg_rl = ChoiceRl(eps_, value_rls_, cost_rls_, engine_);
    int h;

    if (fix) {
      int length = GetRl<int>(arg_rl, ls_);
      h = Walk(h_min, length, state, accepted, best_accepted, sequence);
    } else {
      double rl = GetRl<double>(arg_rl, rls_);
      h = Walk(h_min, rl, state, accepted, best_accepted, sequence);
    }

    ++w;

    int cost = evaluated - before_evaluated;

    if (h < h_min) {
      Feedback(arg_rl, h_min - h, cost, value_rls_, cost_rls_);

      h_min = h;
      current = state;
      plan.insert(plan.end(), sequence.begin(), sequence.end());
      li = w;

      //std::cout << "New best heuristic value for lmcount: " << h
      //          << std::endl;
      //std::cout << "[" << evaluated
      //          << " evaluated, " << expanded << " expanded]" << std::endl;

    } else {
      Feedback(arg_rl, 0, cost, value_rls_, cost_rls_);
    }

    if (h == 0) return plan;

    if (w - li > tg_) {
      ++r;
      double v_i_w = static_cast<double>(initial_h - h_min);
      v_i_w /= static_cast<double>(li);
      v_w += (v_i_w - v_w) / static_cast<double>(r);
      tg_ = initial_h / v_w;
      w = 0;

      h_min = initial_h;
      current = domain_.initial;
      best_accepted = initial_accepted;
      plan.clear();

      //std::cout << "restart new tg=" << tg_ << std::endl;
    }
  }
}

} // namespace rwls
