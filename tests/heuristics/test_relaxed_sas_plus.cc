#include "heuristics/relaxed_sas_plus.h"

#include <algorithm>
#include <queue>
#include <string>

#include "gtest/gtest.h"

#include "sas_plus.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

class RelaxedSASPlusTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    SASPlus sas;
    sas.InitFromLines(lines);

    r_sas_0_ = RelaxedSASPlus(sas, false);
    r_sas_1_ = RelaxedSASPlus(sas, true);
  }

  RelaxedSASPlus r_sas_0_;
  RelaxedSASPlus r_sas_1_;
};

TEST_F(RelaxedSASPlusTest, NFactsWorks) {
  EXPECT_EQ(7, r_sas_0_.n_facts());
  EXPECT_EQ(7, r_sas_1_.n_facts());
}

TEST_F(RelaxedSASPlusTest, NActions) {
  EXPECT_EQ(11, r_sas_0_.n_actions());
  EXPECT_EQ(10, r_sas_1_.n_actions());
}

TEST_F(RelaxedSASPlusTest, NGoalFactsWorks) {
  EXPECT_EQ(1, r_sas_0_.n_goal_facts());
  EXPECT_EQ(1, r_sas_1_.n_goal_facts());
}

TEST_F(RelaxedSASPlusTest, IsGoalWorks) {
  EXPECT_FALSE(r_sas_0_.IsGoal(0));
  EXPECT_FALSE(r_sas_0_.IsGoal(1));
  EXPECT_FALSE(r_sas_0_.IsGoal(2));
  EXPECT_FALSE(r_sas_0_.IsGoal(3));
  EXPECT_FALSE(r_sas_0_.IsGoal(4));
  EXPECT_TRUE(r_sas_0_.IsGoal(5));
  EXPECT_FALSE(r_sas_0_.IsGoal(6));

  EXPECT_FALSE(r_sas_1_.IsGoal(0));
  EXPECT_FALSE(r_sas_1_.IsGoal(1));
  EXPECT_FALSE(r_sas_1_.IsGoal(2));
  EXPECT_FALSE(r_sas_1_.IsGoal(3));
  EXPECT_FALSE(r_sas_1_.IsGoal(4));
  EXPECT_TRUE(r_sas_1_.IsGoal(5));
  EXPECT_FALSE(r_sas_1_.IsGoal(6));
}

TEST_F(RelaxedSASPlusTest, ActionIdWorks) {
  EXPECT_EQ(0, r_sas_0_.ActionId(0));
  EXPECT_EQ(0, r_sas_0_.ActionId(1));
  EXPECT_EQ(1, r_sas_0_.ActionId(2));
  EXPECT_EQ(1, r_sas_0_.ActionId(3));
  EXPECT_EQ(2, r_sas_0_.ActionId(4));
  EXPECT_EQ(3, r_sas_0_.ActionId(5));
  EXPECT_EQ(4, r_sas_0_.ActionId(6));
  EXPECT_EQ(4, r_sas_0_.ActionId(7));
  EXPECT_EQ(5, r_sas_0_.ActionId(8));
  EXPECT_EQ(5, r_sas_0_.ActionId(9));
  EXPECT_EQ(6, r_sas_0_.ActionId(10));

  // Simplify breaks oreder of actions
  //EXPECT_EQ(0, r_sas_1_.ActionId(0));
  //EXPECT_EQ(0, r_sas_1_.ActionId(1));
  //EXPECT_EQ(1, r_sas_1_.ActionId(2));
  //EXPECT_EQ(1, r_sas_1_.ActionId(3));
  //EXPECT_EQ(2, r_sas_1_.ActionId(4));
  //EXPECT_EQ(3, r_sas_1_.ActionId(5));
  //EXPECT_EQ(4, r_sas_1_.ActionId(6));
  //EXPECT_EQ(4, r_sas_1_.ActionId(7));
  //EXPECT_EQ(5, r_sas_1_.ActionId(8));
  //EXPECT_EQ(5, r_sas_1_.ActionId(9));
}

TEST_F(RelaxedSASPlusTest, IdToActionsWorks) {
  auto actions = r_sas_0_.IdToActions(0);
  std::sort(actions.begin(), actions.end());
  EXPECT_EQ(std::vector<int>({0, 1}), actions);
  actions = r_sas_0_.IdToActions(1);
  std::sort(actions.begin(), actions.end());
  EXPECT_EQ(std::vector<int>({2, 3}), actions);
  actions = r_sas_0_.IdToActions(2);
  std::sort(actions.begin(), actions.end());
  EXPECT_EQ(std::vector<int>({4}), actions);
  actions = r_sas_0_.IdToActions(3);
  std::sort(actions.begin(), actions.end());
  EXPECT_EQ(std::vector<int>({5}), actions);
  actions = r_sas_0_.IdToActions(4);
  std::sort(actions.begin(), actions.end());
  EXPECT_EQ(std::vector<int>({6, 7}), actions);
  actions = r_sas_0_.IdToActions(5);
  std::sort(actions.begin(), actions.end());
  EXPECT_EQ(std::vector<int>({8, 9}), actions);
  actions = r_sas_0_.IdToActions(6);
  std::sort(actions.begin(), actions.end());
  EXPECT_EQ(std::vector<int>({10}), actions);
}

TEST_F(RelaxedSASPlusTest, ActionCostWorks) {
  EXPECT_EQ(1, r_sas_0_.ActionCost(0));
  EXPECT_EQ(1, r_sas_0_.ActionCost(1));
  EXPECT_EQ(1, r_sas_0_.ActionCost(2));
  EXPECT_EQ(1, r_sas_0_.ActionCost(3));
  EXPECT_EQ(1, r_sas_0_.ActionCost(4));
  EXPECT_EQ(1, r_sas_0_.ActionCost(5));
  EXPECT_EQ(1, r_sas_0_.ActionCost(6));
  EXPECT_EQ(1, r_sas_0_.ActionCost(7));
  EXPECT_EQ(1, r_sas_0_.ActionCost(9));
  EXPECT_EQ(2, r_sas_0_.ActionCost(10));

  // Simplify breaks oreder of actions
  //EXPECT_EQ(1, r_sas_1_.ActionCost(0));
  //EXPECT_EQ(1, r_sas_1_.ActionCost(1));
  //EXPECT_EQ(1, r_sas_1_.ActionCost(2));
  //EXPECT_EQ(1, r_sas_1_.ActionCost(3));
  //EXPECT_EQ(1, r_sas_1_.ActionCost(4));
  //EXPECT_EQ(1, r_sas_1_.ActionCost(5));
  //EXPECT_EQ(1, r_sas_1_.ActionCost(6));
  //EXPECT_EQ(1, r_sas_1_.ActionCost(7));
  //EXPECT_EQ(1, r_sas_1_.ActionCost(8));
  //EXPECT_EQ(1, r_sas_1_.ActionCost(9));
}

TEST_F(RelaxedSASPlusTest, PreconditionSizeWorks) {
  EXPECT_EQ(2, r_sas_0_.PreconditionSize(0));
  EXPECT_EQ(2, r_sas_0_.PreconditionSize(1));
  EXPECT_EQ(2, r_sas_0_.PreconditionSize(2));
  EXPECT_EQ(2, r_sas_0_.PreconditionSize(3));
  EXPECT_EQ(1, r_sas_0_.PreconditionSize(4));
  EXPECT_EQ(1, r_sas_0_.PreconditionSize(5));
  EXPECT_EQ(3, r_sas_0_.PreconditionSize(6));
  EXPECT_EQ(3, r_sas_0_.PreconditionSize(7));
  EXPECT_EQ(3, r_sas_0_.PreconditionSize(8));
  EXPECT_EQ(3, r_sas_0_.PreconditionSize(9));
  EXPECT_EQ(3, r_sas_0_.PreconditionSize(10));

  // Simplify breaks oreder of actions
  //EXPECT_EQ(2, r_sas_1_.PreconditionSize(0));
  //EXPECT_EQ(2, r_sas_1_.PreconditionSize(1));
  //EXPECT_EQ(2, r_sas_1_.PreconditionSize(2));
  //EXPECT_EQ(2, r_sas_1_.PreconditionSize(3));
  //EXPECT_EQ(1, r_sas_1_.PreconditionSize(4));
  //EXPECT_EQ(1, r_sas_1_.PreconditionSize(5));
  //EXPECT_EQ(3, r_sas_1_.PreconditionSize(6));
  //EXPECT_EQ(3, r_sas_1_.PreconditionSize(7));
  //EXPECT_EQ(3, r_sas_1_.PreconditionSize(8));
  //EXPECT_EQ(3, r_sas_1_.PreconditionSize(9));
}

TEST_F(RelaxedSASPlusTest, PreconditionWorks) {
  auto state = r_sas_0_.Precondition(0);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({0, 2}), state);
  state = r_sas_0_.Precondition(1);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({0, 2}), state);
  state = r_sas_0_.Precondition(2);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({1, 2}), state);
  state = r_sas_0_.Precondition(3);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({1, 2}), state);
  state = r_sas_0_.Precondition(4);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({0}), state);
  state = r_sas_0_.Precondition(5);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({1}), state);
  state = r_sas_0_.Precondition(6);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({0, 3, 4}), state);
  state = r_sas_0_.Precondition(7);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({0, 3, 4}), state);
  state = r_sas_0_.Precondition(8);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({1, 3, 5}), state);
  state = r_sas_0_.Precondition(9);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({1, 3, 5}), state);
  state = r_sas_0_.Precondition(10);
  std::sort(state.begin(), state.end());
  EXPECT_EQ(std::vector<int>({1, 3, 5}), state);

  // Simplify breaks oreder of actions
  //state = r_sas_1_.Precondition(0);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({0, 2}), state);
  //state = r_sas_1_.Precondition(1);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({0, 2}), state);
  //state = r_sas_1_.Precondition(2);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({1, 2}), state);
  //state = r_sas_1_.Precondition(3);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({1, 2}), state);
  //state = r_sas_1_.Precondition(4);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({0}), state);
  //state = r_sas_1_.Precondition(5);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({1}), state);
  //state = r_sas_1_.Precondition(6);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({0, 3, 4}), state);
  //state = r_sas_1_.Precondition(7);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({0, 3, 4}), state);
  //state = r_sas_1_.Precondition(8);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({1, 3, 5}), state);
  //state = r_sas_1_.Precondition(9);
  //std::sort(state.begin(), state.end());
  //EXPECT_EQ(std::vector<int>({1, 3, 5}), state);
}

TEST_F(RelaxedSASPlusTest, PreconditionMapWorks) {
  EXPECT_EQ(std::vector<int>({0, 1, 4, 6, 7}), r_sas_0_.PreconditionMap(0));
  EXPECT_EQ(std::vector<int>({2, 3, 5, 8, 9, 10}), r_sas_0_.PreconditionMap(1));
  EXPECT_EQ(std::vector<int>({0, 1, 2, 3}), r_sas_0_.PreconditionMap(2));
  EXPECT_EQ(std::vector<int>({6, 7, 8, 9, 10}), r_sas_0_.PreconditionMap(3));
  EXPECT_EQ(std::vector<int>({6, 7}), r_sas_0_.PreconditionMap(4));
  EXPECT_EQ(std::vector<int>({8, 9, 10}), r_sas_0_.PreconditionMap(5));
  EXPECT_EQ(std::vector<int>({}), r_sas_0_.PreconditionMap(6));

  // Simplify breaks oreder of actions
  //EXPECT_EQ(std::vector<int>({0, 1, 4, 6, 7}), r_sas_1_.PreconditionMap(0));
  //EXPECT_EQ(std::vector<int>({2, 3, 5, 8, 9, 10}), r_sas_1_.PreconditionMap(1));
  //EXPECT_EQ(std::vector<int>({0, 1, 2, 3}), r_sas_1_.PreconditionMap(2));
  //EXPECT_EQ(std::vector<int>({6, 7, 8, 9, 10}), r_sas_1_.PreconditionMap(3));
  //EXPECT_EQ(std::vector<int>({6, 7}), r_sas_1_.PreconditionMap(4));
  //EXPECT_EQ(std::vector<int>({8, 9, 10}), r_sas_1_.PreconditionMap(5));
  //EXPECT_EQ(std::vector<int>({}), r_sas_1_.PreconditionMap(6));
}

TEST_F(RelaxedSASPlusTest, EffectMapWorks) {
  EXPECT_EQ(std::vector<int>({5}), r_sas_0_.EffectMap(0));
  EXPECT_EQ(std::vector<int>({4}), r_sas_0_.EffectMap(1));
  EXPECT_EQ(std::vector<int>({6, 8, 10}), r_sas_0_.EffectMap(2));
  EXPECT_EQ(std::vector<int>({0, 2}), r_sas_0_.EffectMap(3));
  EXPECT_EQ(std::vector<int>({1}), r_sas_0_.EffectMap(4));
  EXPECT_EQ(std::vector<int>({3}), r_sas_0_.EffectMap(5));
  EXPECT_EQ(std::vector<int>({7, 9}), r_sas_0_.EffectMap(6));

  // Simplify breaks oreder of actions
  //EXPECT_EQ(std::vector<int>({5}), r_sas_1_.EffectMap(0));
  //EXPECT_EQ(std::vector<int>({4}), r_sas_1_.EffectMap(1));
  //EXPECT_EQ(std::vector<int>({7, 9}), r_sas_1_.EffectMap(2));
  //EXPECT_EQ(std::vector<int>({1, 3}), r_sas_1_.EffectMap(3));
  //EXPECT_EQ(std::vector<int>({0}), r_sas_1_.EffectMap(4));
  //EXPECT_EQ(std::vector<int>({2}), r_sas_1_.EffectMap(5));
  //EXPECT_EQ(std::vector<int>({6, 8}), r_sas_1_.EffectMap(6));
}

std::queue<std::string> ExampleSASPlusLines() {
  std::queue<std::string> q;

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push("1");
  q.push("end_metric");
  q.push("3");
  q.push("begin_variable");
  q.push("var0");
  q.push("-1");
  q.push("2");
  q.push("Atom at-robby(rooma)");
  q.push("Atom at-robby(roomb)");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var1");
  q.push("-1");
  q.push("2");
  q.push("Atom carry(ball1, left)");
  q.push("Atom free(left)");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var2");
  q.push("-1");
  q.push("3");
  q.push("Atom at(ball1, rooma)");
  q.push("Atom at(ball1, roomb)");
  q.push("<none of those>");
  q.push("end_variable");
  q.push("1");
  q.push("begin_mutex_group");
  q.push("3");
  q.push("2 0");
  q.push("2 1");
  q.push("1 0");
  q.push("end_mutex_group");
  q.push("begin_state");
  q.push("0");
  q.push("1");
  q.push("0");
  q.push("end_state");
  q.push("begin_goal");
  q.push("1");
  q.push("2 1");
  q.push("end_goal");
  q.push("7");
  q.push("begin_operator");
  q.push("drop ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 -1 0");
  q.push("0 1 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("drop ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 -1 1");
  q.push("0 1 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move rooma roomb");
  q.push("0");
  q.push("1");
  q.push("0 0 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move roomb rooma");
  q.push("0");
  q.push("1");
  q.push("0 0 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 0 2");
  q.push("0 1 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 1 2");
  q.push("0 1 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("dumy-pick ball1 roomb left");
  q.push("2");
  q.push("0 1");
  q.push("2 1");
  q.push("1");
  q.push("0 1 1 0");
  q.push("2");
  q.push("end_operator");
  q.push("0");

  return q;
}

} // namespace pplanner
