#include "domain/state.h"

#include <cassert>

#include <iostream>
#include <vector>

using namespace rwls;

void TestStateHash() {
  State s(6, 0);
  State t(s);

  StateHash hash;

  assert(hash(t) == hash(s));
  s[0] = 0;
  t[0] = 0;
  assert(hash(t) == hash(s));

  std::cout << "passed StateHash" << std::endl;
}

void TestApplyEffect() {
  std::vector<VarValue> effect(2);
  EncodeVarValue(0, 1, &effect[0]);
  EncodeVarValue(3, 2, &effect[1]);
  State state{3, 2, 1, 4};
  ApplyEffect(effect, state);
  assert(1 == state[0]);
  assert(2 == state[3]);

  std::cout << "passed ApplyEffect" << std::endl;
}

void TestGoalCheck() {
  std::vector<VarValue> goal(2);
  EncodeVarValue(0, 1, &goal[0]);
  EncodeVarValue(3, 2, &goal[1]);
  State state{3, 2, 1, 4};
  assert(!GoalCheck(goal, state));
  state[0] = 1;
  assert(!GoalCheck(goal, state));
  state[3] = 2;
  assert(GoalCheck(goal, state));

  std::cout << "passed GoalCheck" << std::endl;
}

int main() {
  TestStateHash();
  TestApplyEffect();
  TestGoalCheck();
}
