#include "successor_generator.h"

#include <algorithm>
#include <queue>
#include <string>

#include "gtest/gtest.h"

#include "sas_plus.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

std::queue<std::string> NoPickSASPlusLines();

class SuccessorGeneratorTest: public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    auto sas_0 = std::make_shared<SASPlus>();
    sas_0->InitFromLines(lines);
    generator_0_ = SuccessorGenerator(sas_0);
    //generator_0_.Dump();

    lines = NoPickSASPlusLines();
    auto sas_1 = std::make_shared<SASPlus>();
    sas_1->InitFromLines(lines);
    generator_1_ = SuccessorGenerator(sas_1);
    //generator_1_.Dump();
  }

  SuccessorGenerator generator_0_;
  SuccessorGenerator generator_1_;
};

TEST_F(SuccessorGeneratorTest, GenerateWorks) {
  std::vector<int> state{0, 1, 0};
  std::vector<int> result;

  generator_1_.Generate(state, result);
  EXPECT_TRUE(result.empty());

  generator_0_.Generate(state, result);
  ASSERT_EQ(2, result.size());
  std::sort(result.begin(), result.end());
  EXPECT_EQ(2, result[0]);
  EXPECT_EQ(4, result[1]);

  state[1] = 0;
  state[2] = 2;
  generator_0_.Generate(state, result);
  ASSERT_EQ(2, result.size());
  std::sort(result.begin(), result.end());
  EXPECT_EQ(0, result[0]);
  EXPECT_EQ(2, result[1]);

  state[0] = 1;
  generator_0_.Generate(state, result);
  ASSERT_EQ(2, result.size());
  std::sort(result.begin(), result.end());
  EXPECT_EQ(1, result[0]);
  EXPECT_EQ(3, result[1]);
}

TEST_F(SuccessorGeneratorTest, SampleWorks) {
  std::vector<int> state{0, 1, 0};
  std::vector<int> result;

  int a = generator_1_.Sample(state);
  EXPECT_EQ(-1, a);

  a = generator_0_.Sample(state);
  EXPECT_TRUE(2 == a || 4 == a);

  state[1] = 0;
  state[2] = 2;
  a = generator_0_.Sample(state);
  EXPECT_TRUE(0 == a || 2 == a);

  state[0] = 1;
  a = generator_0_.Sample(state);
  EXPECT_TRUE(1 == a || 3 == a);
}

std::queue<std::string> ExampleSASPlusLines() {
  std::queue<std::string> q;

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push("0");
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
  q.push("0");

  return q;
}

std::queue<std::string> NoPickSASPlusLines() {
  std::queue<std::string> q;

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push("0");
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
  q.push("3");
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
  q.push("move roomb rooma");
  q.push("0");
  q.push("1");
  q.push("0 0 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("0");

  return q;
}

} // namespace pplanner
