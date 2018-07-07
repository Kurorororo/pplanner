#include "hash/zobrist_hash.h"

#include "gtest/gtest.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

class ZobristHashTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    sas_ = std::make_shared<SASPlus>();
    sas_->InitFromLines(lines);
    hash_ = std::make_shared<ZobristHash>(sas_, 123);
  }

  std::shared_ptr<ZobristHash> hash_;
  std::shared_ptr<SASPlus> sas_;
};

TEST_F(ZobristHashTest, Works) {
  std::vector<int> state_0{0, 1, 0};
  std::vector<int> state_1{0, 1, 0};
  std::vector<int> state_2{0, 0, 0};
  std::vector<int> state_3{0, 0, 0};

  EXPECT_EQ(hash_->operator()(state_0), hash_->operator()(state_1));
  EXPECT_EQ(hash_->operator()(state_2), hash_->operator()(state_3));
}

TEST_F(ZobristHashTest, HashByDifferenceWorks) {
  std::vector<int> state_0{0, 1, 0};
  std::vector<int> state_1(state_0);

  uint32_t seed = hash_->operator()(state_0);
  sas_->ApplyEffect(4, state_1);
  EXPECT_EQ(hash_->operator()(state_1),
            hash_->HashByDifference(4, seed, state_0, state_1));
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

} // namespace pplanner
