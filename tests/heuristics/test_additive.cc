#include "heuristics/additive.h"

#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"

#include "sas_plus.h"
#include "heuristics/relaxed_sas_plus.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

class AdditiveTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    sas_ = std::make_shared<SASPlus>();
    sas_->InitFromLines(lines);

    additive_0_ = std::make_shared<Additive>(sas_);
    additive_1_ = std::make_shared<Additive>(sas_);
  }

  std::shared_ptr<SASPlus> sas_;
  std::shared_ptr<Additive> additive_0_;
  std::shared_ptr<Additive> additive_1_;
};

TEST_F(AdditiveTest, EvaluateWorks) {
  int node = 0;
  std::vector<int> state{0, 1, 0};
  EXPECT_EQ(3, additive_0_->Evaluate(state, node++));
  state = {0, 0, 2};
  EXPECT_EQ(2, additive_0_->Evaluate(state, node++));
  state = {1, 1, 0};
  EXPECT_EQ(3, additive_0_->Evaluate(state, node++));
  state = {1, 0, 2};
  EXPECT_EQ(1, additive_0_->Evaluate(state, node++));
  state = {1, 1, 1};
  EXPECT_EQ(0, additive_0_->Evaluate(state, node++));

  state = {0, 1, 0};
  EXPECT_EQ(3, additive_1_->Evaluate(state, node++));
  state = {0, 0, 2};
  EXPECT_EQ(2, additive_1_->Evaluate(state, node++));
  state = {1, 1, 0};
  EXPECT_EQ(3, additive_1_->Evaluate(state, node++));
  state = {1, 0, 2};
  EXPECT_EQ(1, additive_1_->Evaluate(state, node++));
  state = {1, 1, 1};
  EXPECT_EQ(0, additive_1_->Evaluate(state, node++));
}

TEST_F(AdditiveTest, EvaluateWithPreferredWorks) {
  int node = 0;
  std::vector<int> state{0, 1, 0};
  std::vector<int> applicable;
  std::unordered_set<int> preferred;
  EXPECT_EQ(3, additive_0_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());
  state = {0, 0, 2};
  EXPECT_EQ(2, additive_0_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());
  state = {1, 1, 0};
  EXPECT_EQ(3, additive_0_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());
  state = {1, 0, 2};
  EXPECT_EQ(1, additive_0_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());
  state = {1, 1, 1};
  EXPECT_EQ(0, additive_0_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());

  state = {0, 1, 0};
  EXPECT_EQ(3, additive_1_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());
  state = {0, 0, 2};
  EXPECT_EQ(2, additive_1_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());
  state = {1, 1, 0};
  EXPECT_EQ(3, additive_1_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());
  state = {1, 0, 2};
  EXPECT_EQ(1, additive_1_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());
  state = {1, 1, 1};
  EXPECT_EQ(0, additive_1_->Evaluate(state, node++, applicable, preferred));
  EXPECT_TRUE(preferred.empty());
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
