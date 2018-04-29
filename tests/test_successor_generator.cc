#include "successor_generator.h"

#include <algorithm>
#include <queue>
#include <string>

#include "gtest/gtest.h"

#include "sas_plus.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines(bool unit_cost=true) {
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

class SuccessorGeneratorTest: public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    SASPlus sas;
    sas.InitFromLines(lines);
    generator_ = SuccessorGenerator(sas);
    //generator_.Dump();
  }

  SuccessorGenerator generator_;
};

TEST_F(SuccessorGeneratorTest, GenerateWorks) {
  std::vector<int> state{0, 1, 0};
  std::vector<int> result;
  generator_.Generate(state, result);
  ASSERT_EQ(2, result.size());
  std::sort(result.begin(), result.end());
  EXPECT_EQ(2, result[0]);
  EXPECT_EQ(4, result[1]);

  state[1] = 0;
  state[2] = 2;
  generator_.Generate(state, result);
  ASSERT_EQ(2, result.size());
  std::sort(result.begin(), result.end());
  EXPECT_EQ(0, result[0]);
  EXPECT_EQ(2, result[1]);

  state[0] = 1;
  generator_.Generate(state, result);
  ASSERT_EQ(2, result.size());
  std::sort(result.begin(), result.end());
  EXPECT_EQ(1, result[0]);
  EXPECT_EQ(3, result[1]);
}

TEST_F(SuccessorGeneratorTest, SampleWorks) {
  std::vector<int> state{0, 1, 0};
  std::vector<int> result;
  int a = generator_.Sample(state);
  EXPECT_TRUE(2 == a || 4 == a);

  state[1] = 0;
  state[2] = 2;
  a = generator_.Sample(state);
  EXPECT_TRUE(0 == a || 2 == a);

  state[0] = 1;
  a = generator_.Sample(state);
  EXPECT_TRUE(1 == a || 3 == a);
}

} // namespace pplanner
