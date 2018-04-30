#include "search_graph/state_packer.h"

#include <cstdio>

#include <queue>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

std::queue<std::string> RandomSASPlusLines(std::vector<int> &ranges);

class StatePackerTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    SASPlus sas_0;
    sas_0.InitFromLines(lines);
    packer_0_ = StatePacker(sas_0);

    state_0_ = std::vector<int>{1, 0, 2};

    std::vector<int> ranges;
    lines = RandomSASPlusLines(ranges);
    SASPlus sas_1;
    sas_1.InitFromLines(lines);
    packer_1_ = StatePacker(sas_1);

    for (auto range : ranges)
      state_1_.push_back(rand() % range);
  }

  StatePacker packer_0_;
  StatePacker packer_1_;
  StatePacker packer_2_;
  std::vector<int> state_0_;
  std::vector<int> state_1_;
};

TEST_F(StatePackerTest, BlockSizeWorks) {
  EXPECT_EQ(1, packer_0_.block_size());
  EXPECT_EQ(0, packer_2_.block_size());
}

TEST_F(StatePackerTest, PackUnPackWorks) {
  std::vector<int> tmp_state(state_0_.size());
  std::vector<uint32_t> tmp_packed(packer_0_.block_size());

  packer_0_.Pack(state_0_, tmp_packed.data());
  packer_0_.Unpack(tmp_packed.data(), tmp_state);
  EXPECT_EQ(state_0_, tmp_state);

  tmp_state.resize(state_1_.size());
  tmp_packed.resize(packer_1_.block_size());
  packer_1_.Pack(state_1_, tmp_packed.data());
  packer_1_.Unpack(tmp_packed.data(), tmp_state);
  EXPECT_EQ(state_1_, tmp_state);
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
