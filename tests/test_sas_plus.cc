#include "sas_plus.h"

#include <algorithm>
#include <queue>
#include <string>

#include "gtest/gtest.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines(bool unit_cost=true);

class SASPlusTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    sas_1_.InitFromLines(lines);
  }

  SASPlus sas_0_;
  SASPlus sas_1_;
};

TEST_F(SASPlusTest, IsInitialized) {
  EXPECT_EQ(0, sas_0_.metric());
  EXPECT_EQ(nullptr, sas_0_.facts());
  EXPECT_EQ(nullptr, sas_0_.goal());
  EXPECT_EQ(nullptr, sas_0_.preconditions());
  EXPECT_EQ(nullptr, sas_0_.effects());
}

TEST_F(SASPlusTest, InitFromLinesWorks) {
  auto lines =  ExampleSASPlusLines();
  sas_0_.InitFromLines(lines);
  EXPECT_EQ(0, sas_0_.metric());
  EXPECT_NE(nullptr, sas_0_.facts());
  EXPECT_NE(nullptr, sas_0_.goal());
  EXPECT_NE(nullptr, sas_0_.preconditions());
  EXPECT_NE(nullptr, sas_0_.effects());
}

TEST_F(SASPlusTest, NVariablesWorks) {
  EXPECT_EQ(3, sas_1_.n_variables());
}

TEST_F(SASPlusTest, NGoalFactsWorks) {
  EXPECT_EQ(1, sas_1_.n_goal_facts());
}

TEST_F(SASPlusTest, NActionsWorks) {
  EXPECT_EQ(6, sas_1_.n_actions());
}

TEST_F(SASPlusTest, MetricWorks) {
  EXPECT_EQ(0, sas_1_.metric());
  auto lines = ExampleSASPlusLines(false);
  sas_0_.InitFromLines(lines);
  EXPECT_EQ(1, sas_0_.metric());
}

TEST_F(SASPlusTest, InitialWorks) {
  auto state = sas_1_.initial();
  EXPECT_EQ(0, state[0]);
  EXPECT_EQ(1, state[1]);
  EXPECT_EQ(0, state[2]);
}

TEST_F(SASPlusTest, FactWorks) {
  EXPECT_EQ(0, sas_1_.Fact(0, 0));
  EXPECT_EQ(1, sas_1_.Fact(0, 1));
  EXPECT_EQ(2, sas_1_.Fact(1, 0));
  EXPECT_EQ(3, sas_1_.Fact(1, 1));
  EXPECT_EQ(4, sas_1_.Fact(2, 0));
  EXPECT_EQ(5, sas_1_.Fact(2, 1));
  EXPECT_EQ(6, sas_1_.Fact(2, 2));
}

TEST_F(SASPlusTest, VarBeginWorks) {
  EXPECT_EQ(0, sas_1_.VarBegin(0));
  EXPECT_EQ(2, sas_1_.VarBegin(1));
  EXPECT_EQ(4, sas_1_.VarBegin(2));
}

TEST_F(SASPlusTest, VarRangeWorks) {
  EXPECT_EQ(2, sas_1_.VarRange(0));
  EXPECT_EQ(2, sas_1_.VarRange(1));
  EXPECT_EQ(3, sas_1_.VarRange(2));
}

TEST_F(SASPlusTest, PredicateWorks) {
  EXPECT_EQ(std::string("at-robby"), sas_1_.Predicate(0, 0));
  EXPECT_EQ(std::string("at-robby"), sas_1_.Predicate(0, 1));
  EXPECT_EQ(std::string("carry"), sas_1_.Predicate(1, 0));
  EXPECT_EQ(std::string("free"), sas_1_.Predicate(1, 1));
  EXPECT_EQ(std::string("at"), sas_1_.Predicate(2, 0));
  EXPECT_EQ(std::string("at"), sas_1_.Predicate(2, 1));
  EXPECT_EQ(std::string("<none of those>"), sas_1_.Predicate(2, 2));
}

TEST_F(SASPlusTest, IsMutexWorks) {
  EXPECT_FALSE(sas_1_.IsMutex(3, 5));
  EXPECT_FALSE(sas_1_.IsMutex(1, 6));
  EXPECT_FALSE(sas_1_.IsMutex(0, 4));
  EXPECT_TRUE(sas_1_.IsMutex(4, 5));
  EXPECT_TRUE(sas_1_.IsMutex(2, 5));
  EXPECT_TRUE(sas_1_.IsMutex(2, 4));

  EXPECT_FALSE(sas_1_.IsMutex(2, 0, 3, 0));
  EXPECT_FALSE(sas_1_.IsMutex(1, 0, 1, 1));
  EXPECT_FALSE(sas_1_.IsMutex(0, 1, 2, 1));
  EXPECT_TRUE(sas_1_.IsMutex(2, 0, 2, 1));
  EXPECT_TRUE(sas_1_.IsMutex(1, 0, 2, 0));
  EXPECT_TRUE(sas_1_.IsMutex(1, 0, 2, 1));
}

TEST_F(SASPlusTest, IsGoalWorks) {
  auto state = sas_1_.initial();
  EXPECT_FALSE(sas_1_.IsGoal(state));
  state[2] = 1;
  EXPECT_TRUE(sas_1_.IsGoal(state));
}

TEST_F(SASPlusTest, ActionCostWorks) {
  EXPECT_EQ(1, sas_1_.ActionCost(0));
  EXPECT_EQ(1, sas_1_.ActionCost(1));
  EXPECT_EQ(1, sas_1_.ActionCost(2));
  EXPECT_EQ(1, sas_1_.ActionCost(3));
  EXPECT_EQ(1, sas_1_.ActionCost(4));
  EXPECT_EQ(1, sas_1_.ActionCost(5));

  auto lines = ExampleSASPlusLines(false);
  sas_0_.InitFromLines(lines);
  EXPECT_EQ(10, sas_0_.ActionCost(0));
  EXPECT_EQ(10, sas_0_.ActionCost(1));
  EXPECT_EQ(10, sas_0_.ActionCost(2));
  EXPECT_EQ(10, sas_0_.ActionCost(3));
  EXPECT_EQ(10, sas_0_.ActionCost(4));
  EXPECT_EQ(10, sas_0_.ActionCost(5));
}

TEST_F(SASPlusTest, ActionNameWorks) {
  EXPECT_EQ(std::string("drop ball1 rooma left"), sas_1_.ActionName(0));
  EXPECT_EQ(std::string("drop ball1 roomb left"), sas_1_.ActionName(1));
  EXPECT_EQ(std::string("move rooma roomb"), sas_1_.ActionName(2));
  EXPECT_EQ(std::string("move roomb rooma"), sas_1_.ActionName(3));
  EXPECT_EQ(std::string("pick ball1 rooma left"), sas_1_.ActionName(4));
  EXPECT_EQ(std::string("pick ball1 roomb left"), sas_1_.ActionName(5));
}

TEST_F(SASPlusTest, CopyPreconditionWorks) {
  std::vector<std::pair<int, int> > precondition;

  sas_1_.CopyPrecondition(0, precondition);
  std::sort(precondition.begin(), precondition.end());
  ASSERT_EQ(2, precondition.size());
  EXPECT_EQ(0, precondition[0].first);
  EXPECT_EQ(0, precondition[0].second);
  EXPECT_EQ(1, precondition[1].first);
  EXPECT_EQ(0, precondition[1].second);

  sas_1_.CopyPrecondition(1, precondition);
  std::sort(precondition.begin(), precondition.end());
  ASSERT_EQ(2, precondition.size());
  EXPECT_EQ(0, precondition[0].first);
  EXPECT_EQ(1, precondition[0].second);
  EXPECT_EQ(1, precondition[1].first);
  EXPECT_EQ(0, precondition[1].second);

  sas_1_.CopyPrecondition(2, precondition);
  std::sort(precondition.begin(), precondition.end());
  ASSERT_EQ(1, precondition.size());
  EXPECT_EQ(0, precondition[0].first);
  EXPECT_EQ(0, precondition[0].second);

  sas_1_.CopyPrecondition(3, precondition);
  std::sort(precondition.begin(), precondition.end());
  ASSERT_EQ(1, precondition.size());
  EXPECT_EQ(0, precondition[0].first);
  EXPECT_EQ(1, precondition[0].second);

  sas_1_.CopyPrecondition(4, precondition);
  std::sort(precondition.begin(), precondition.end());
  ASSERT_EQ(3, precondition.size());
  EXPECT_EQ(0, precondition[0].first);
  EXPECT_EQ(0, precondition[0].second);
  EXPECT_EQ(1, precondition[1].first);
  EXPECT_EQ(1, precondition[1].second);
  EXPECT_EQ(2, precondition[2].first);
  EXPECT_EQ(0, precondition[2].second);

  sas_1_.CopyPrecondition(5, precondition);
  std::sort(precondition.begin(), precondition.end());
  ASSERT_EQ(3, precondition.size());
  EXPECT_EQ(0, precondition[0].first);
  EXPECT_EQ(1, precondition[0].second);
  EXPECT_EQ(1, precondition[1].first);
  EXPECT_EQ(1, precondition[1].second);
  EXPECT_EQ(2, precondition[2].first);
  EXPECT_EQ(1, precondition[2].second);
}

TEST_F(SASPlusTest, ApplyEffectWorks) {
  auto state = sas_1_.initial();
  sas_1_.ApplyEffect(4, state);
  EXPECT_EQ(0, state[1]);
  sas_1_.ApplyEffect(2, state);
  EXPECT_EQ(1, state[0]);
  sas_1_.ApplyEffect(1, state);
  EXPECT_EQ(1, state[2]);
}


TEST_F(SASPlusTest, StateToFactVectorWorks) {
  std::vector<int> v;
  auto state = sas_1_.initial();
  StateToFactVector(sas_1_, state, v);
  ASSERT_TRUE(state.size() == v.size());
  EXPECT_EQ(0, v[0]);
  EXPECT_EQ(3, v[1]);
  EXPECT_EQ(4, v[2]);
  state[0] = 1;
  StateToFactVector(sas_1_, state, v);
  ASSERT_TRUE(state.size() == v.size());
  EXPECT_EQ(1, v[0]);
  EXPECT_EQ(3, v[1]);
  EXPECT_EQ(4, v[2]);
}

TEST_F(SASPlusTest, StateToFactSetWorks) {
  std::vector<bool> s;
  auto state = sas_1_.initial();
  StateToFactSet(sas_1_, state, s);
  ASSERT_TRUE(sas_1_.n_facts() == s.size());
  EXPECT_TRUE(s[0]);
  EXPECT_FALSE(s[1]);
  EXPECT_FALSE(s[2]);
  EXPECT_TRUE(s[3]);
  EXPECT_TRUE(s[4]);
  EXPECT_FALSE(s[5]);
  state[0] = 1;
  StateToFactSet(sas_1_, state, s);
  ASSERT_TRUE(sas_1_.n_facts() == s.size());
  EXPECT_FALSE(s[0]);
  EXPECT_TRUE(s[1]);
  EXPECT_FALSE(s[2]);
  EXPECT_TRUE(s[3]);
  EXPECT_TRUE(s[4]);
  EXPECT_FALSE(s[5]);
}

std::queue<std::string> ExampleSASPlusLines(bool unit_cost) {
  std::queue<std::string> q;

  std::string cost = "10";
  std::string metric = unit_cost ? "0" : "1";

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push(metric);
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
  q.push("6");
  q.push("begin_operator");
  q.push("drop ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 -1 0");
  q.push("0 1 0 1");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("drop ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 -1 1");
  q.push("0 1 0 1");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move rooma roomb");
  q.push("0");
  q.push("1");
  q.push("0 0 0 1");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move roomb rooma");
  q.push("0");
  q.push("1");
  q.push("0 0 1 0");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 0 2");
  q.push("0 1 1 0");
  q.push(cost);
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 1 2");
  q.push("0 1 1 0");
  q.push(cost);
  q.push("end_operator");
  q.push("0");

  return q;
}

} // namespace pplanner
