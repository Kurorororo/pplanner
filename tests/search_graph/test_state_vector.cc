#include "search_graph/state_vector.h"

#include <cstdio>

#include <queue>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

std::queue<std::string> RandomSASPlusLines(std::vector<int> &ranges);

class StateVectorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    SASPlus sas_0;
    sas_0.InitFromLines(lines);
    vector_0_ = StateVector(sas_0);

    state_0_ = std::vector<int>{1, 0, 2};

    std::vector<int> ranges;
    lines = RandomSASPlusLines(ranges);
    SASPlus sas_1;
    sas_1.InitFromLines(lines);
    vector_1_ = StateVector(sas_1);

    for (auto range : ranges)
      state_1_.push_back(rand() % range);

    state_1_[0] = 1;
    state_1_[1] = 0;
    state_1_[2] = 2;

    vector_2_ = StateVector(sas_0, 2);
  }

  StateVector vector_0_;
  StateVector vector_1_;
  StateVector vector_2_;
  std::vector<int> state_0_;
  std::vector<int> state_1_;
};

TEST_F(StateVectorTest, SizeWorks) {
  EXPECT_EQ(0, vector_0_.size());
  EXPECT_EQ(0, vector_1_.size());
}

TEST_F(StateVectorTest, StateSizeWorks) {
  EXPECT_EQ(1 * sizeof(uint32_t), vector_0_.state_size());
}

TEST_F(StateVectorTest, AddWorks) {
  EXPECT_EQ(0, vector_0_.Add(state_0_));
  EXPECT_EQ(1, vector_0_.size());
  EXPECT_EQ(1, vector_0_.Add(state_0_));
  EXPECT_EQ(2, vector_0_.size());
  EXPECT_EQ(0, vector_1_.Add(state_1_));
  EXPECT_EQ(1, vector_1_.size());
  EXPECT_EQ(1, vector_1_.Add(state_1_));
  EXPECT_EQ(2, vector_1_.size());
}

TEST_F(StateVectorTest, GetWorks) {
  int i = vector_0_.Add(state_0_);
  std::vector<int> tmp(state_0_.size());
  vector_0_.Get(i, tmp);
  EXPECT_EQ(state_0_, tmp);
  i = vector_0_.Add(state_0_);
  vector_0_.Get(i, tmp);
  EXPECT_EQ(state_0_, tmp);

  i = vector_1_.Add(state_1_);
  tmp.resize(state_1_.size());
  vector_1_.Get(i, tmp);
  EXPECT_EQ(state_1_, tmp);
  i = vector_1_.Add(state_1_);
  vector_1_.Get(i, tmp);
  EXPECT_EQ(state_1_, tmp);
}

TEST_F(StateVectorTest, ClosedSizeWorks) {
  EXPECT_EQ((1 << 22) * sizeof(int), vector_0_.closed_size());
  EXPECT_EQ((1 << 22) * sizeof(int), vector_1_.closed_size());
  EXPECT_EQ((1 << 2) * sizeof(int), vector_2_.closed_size());
}

TEST_F(StateVectorTest, CloseWorks) {
  int i = vector_2_.Add(state_0_);
  vector_2_.Close(i);
  i = vector_2_.Add(state_0_);
  vector_2_.Close(i);
  i = vector_2_.Add(state_0_);
  vector_2_.Close(i);
  i = vector_2_.Add(state_0_);
  vector_2_.Close(i);
  EXPECT_GE(vector_2_.closed_size(), 4);
}

TEST_F(StateVectorTest, GetClosedWorks) {
  EXPECT_EQ(-1, vector_0_.GetClosed(state_0_));
  int i = vector_0_.Add(state_0_);
  vector_0_.Close(i);
  EXPECT_EQ(i, vector_0_.GetClosed(state_0_));
  std::vector<int> tmp_state_0(state_0_);
  tmp_state_0[1] = 1;
  EXPECT_EQ(-1, vector_0_.GetClosed(tmp_state_0));
  i = vector_0_.Add(tmp_state_0);
  vector_0_.Close(i);
  EXPECT_EQ(i, vector_0_.GetClosed(tmp_state_0));

  EXPECT_EQ(-1, vector_1_.GetClosed(state_1_));
  i = vector_1_.Add(state_1_);
  vector_1_.Close(i);
  EXPECT_EQ(i, vector_1_.GetClosed(state_1_));
  std::vector<int> tmp_state_1(state_1_);
  tmp_state_1[1] = 1;
  EXPECT_EQ(-1, vector_1_.GetClosed(tmp_state_1));
  i = vector_1_.Add(tmp_state_1);
  vector_1_.Close(i);
  EXPECT_EQ(i, vector_1_.GetClosed(tmp_state_1));
}

TEST_F(StateVectorTest, AddIfNotClosedWorks) {
  int i = vector_0_.AddIfNotClosed(state_0_);
  EXPECT_EQ(0, i);
  vector_0_.Close(i);
  i = vector_0_.AddIfNotClosed(state_0_);
  EXPECT_EQ(-1, i);
  std::vector<int> tmp_state_0(state_0_);
  tmp_state_0[1] = 1;
  i = vector_0_.AddIfNotClosed(tmp_state_0);
  EXPECT_EQ(1, i);
  vector_0_.Close(i);
  i = vector_0_.AddIfNotClosed(tmp_state_0);
  EXPECT_EQ(-1, i);

  i = vector_1_.AddIfNotClosed(state_1_);
  EXPECT_EQ(0, i);
  vector_1_.Close(i);
  i = vector_1_.AddIfNotClosed(state_1_);
  EXPECT_EQ(-1, i);
  std::vector<int> tmp_state_1(state_1_);
  tmp_state_1[1] = 1;
  i = vector_1_.AddIfNotClosed(tmp_state_1);
  EXPECT_EQ(1, i);
  vector_1_.Close(i);
  i = vector_1_.AddIfNotClosed(tmp_state_1);
  EXPECT_EQ(-1, i);
}

TEST_F(StateVectorTest, GetStateAndClosedWorks) {
  int i = vector_0_.Add(state_0_);
  std::vector<int> tmp_state_0(state_0_.size());
  EXPECT_EQ(-1, vector_0_.GetStateAndClosed(i, tmp_state_0));
  EXPECT_EQ(state_0_, tmp_state_0);
  vector_0_.Close(i);
  EXPECT_EQ(i, vector_0_.GetStateAndClosed(i, tmp_state_0));
  state_0_[1] = 1;
  i = vector_0_.Add(state_0_);
  EXPECT_EQ(-1, vector_0_.GetStateAndClosed(i, tmp_state_0));
  EXPECT_EQ(state_0_, tmp_state_0);
  vector_0_.Close(i);
  EXPECT_EQ(i, vector_0_.GetStateAndClosed(i, tmp_state_0));

  i = vector_1_.Add(state_1_);
  std::vector<int> tmp_state_1(state_1_.size());
  EXPECT_EQ(-1, vector_1_.GetStateAndClosed(i, tmp_state_1));
  EXPECT_EQ(state_1_, tmp_state_1);
  vector_1_.Close(i);
  EXPECT_EQ(i, vector_1_.GetStateAndClosed(i, tmp_state_1));
  state_1_[1] = 1;
  i = vector_1_.Add(state_1_);
  EXPECT_EQ(-1, vector_1_.GetStateAndClosed(i, tmp_state_1));
  EXPECT_EQ(state_1_, tmp_state_1);
  vector_1_.Close(i);
  EXPECT_EQ(i, vector_1_.GetStateAndClosed(i, tmp_state_1));
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

std::queue<std::string> RandomSASPlusLines(std::vector<int> &ranges) {
  std::queue<std::string> q;

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push("0");
  q.push("end_metric");

  int var_max = rand() % 200 + 3;
  q.push(std::to_string(var_max));

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

  ranges = std::vector<int>{2, 2, 3};

  for (int i=3; i<var_max; ++i) {
    q.push("begin_variable");
    q.push("var" + std::to_string(i));
    q.push("-1");

    int range = rand() % 20 + 1;
    ranges.push_back(range);
    q.push(std::to_string(range));

    for (int j=0; j<range; ++j)
      q.push("Atom dummy(ball" + std::to_string(j) + ")");

    q.push("end_variable");
  }

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

  for (int i=3; i<var_max; ++i)
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

} // namespace pplanner
