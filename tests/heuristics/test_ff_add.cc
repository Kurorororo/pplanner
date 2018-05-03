#include "heuristics/ff_add.h"

#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"

#include "sas_plus.h"
#include "heuristics/additive.h"
#include "heuristics/relaxed_sas_plus.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

class FFAddTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    sas_ = std::make_shared<SASPlus>();
    sas_->InitFromLines(lines);

    additive_0_ = std::make_shared<Additive>(sas_);
    additive_1_ = std::make_shared<Additive>(sas_);

    ff_0_ = std::make_shared<FFAdd>(sas_);
    ff_1_ = std::make_shared<FFAdd>(sas_);
  }

  std::shared_ptr<SASPlus> sas_;
  std::shared_ptr<Additive> additive_0_;
  std::shared_ptr<Additive> additive_1_;
  std::shared_ptr<FFAdd> ff_0_;
  std::shared_ptr<FFAdd> ff_1_;
};

TEST_F(FFAddTest, EvaluateWorks) {
  int node = 0;
  std::vector<int> state{0, 1, 0};
  int h = ff_0_->Evaluate(state, node++);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_0_->Evaluate(state, node++), h);
  state = {0, 0, 2};
  h = ff_0_->Evaluate(state, node++);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_0_->Evaluate(state, node++), h);
  state = {1, 1, 0};
  h = ff_0_->Evaluate(state, node++);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_0_->Evaluate(state, node++), h);
  state = {1, 0, 2};
  h = ff_0_->Evaluate(state, node++);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_0_->Evaluate(state, node++), h);
  state = {1, 1, 1};
  h = ff_0_->Evaluate(state, node++);
  EXPECT_EQ(0, h);

  state = {0, 1, 0};
  h = ff_1_->Evaluate(state, node++);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_1_->Evaluate(state, node++), h);
  state = {0, 0, 2};
  h = ff_1_->Evaluate(state, node++);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_1_->Evaluate(state, node++), h);
  state = {1, 1, 0};
  h = ff_1_->Evaluate(state, node++);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_1_->Evaluate(state, node++), h);
  state = {1, 0, 2};
  h = ff_1_->Evaluate(state, node++);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_1_->Evaluate(state, node++), h);
  state = {1, 1, 1};
  h = ff_1_->Evaluate(state, node++);
  EXPECT_EQ(0, h);
}

TEST_F(FFAddTest, EvaluateWithPreferredWorks) {
  int node = 0;
  std::vector<int> state;
  std::vector<int> applicable;
  std::unordered_set<int> preferred;

  state = {0, 1, 0};
  applicable = {2, 4};
  int h = ff_0_->Evaluate(state, node++, applicable, preferred);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_0_->Evaluate(state, node++), h);
  EXPECT_FALSE(preferred.empty());

  state = {0, 0, 2};
  applicable = {0, 2};
  h = ff_0_->Evaluate(state, node++, applicable, preferred);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_0_->Evaluate(state, node++), h);
  EXPECT_FALSE(preferred.empty());

  state = {1, 0, 2};
  applicable = {1, 3};
  h = ff_0_->Evaluate(state, node++, applicable, preferred);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_0_->Evaluate(state, node++), h);
  EXPECT_FALSE(preferred.empty());

  state = {1, 1, 1};
  applicable = {3, 5, 6};
  h = ff_0_->Evaluate(state, node++, applicable, preferred);
  EXPECT_EQ(0, h);
  EXPECT_TRUE(preferred.empty());

  state = {0, 1, 0};
  applicable = {2, 4};
  h = ff_1_->Evaluate(state, node++, applicable, preferred);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_1_->Evaluate(state, node++), h);
  EXPECT_FALSE(preferred.empty());

  state = {0, 0, 2};
  applicable = {0, 2};
  h = ff_1_->Evaluate(state, node++, applicable, preferred);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_1_->Evaluate(state, node++), h);
  EXPECT_FALSE(preferred.empty());

  state = {1, 0, 2};
  applicable = {1, 3};
  h = ff_1_->Evaluate(state, node++, applicable, preferred);
  EXPECT_LT(0, h);
  EXPECT_GE(additive_1_->Evaluate(state, node++), h);
  EXPECT_FALSE(preferred.empty());

  state = {1, 1, 1};
  applicable = {3, 5};
  h = ff_1_->Evaluate(state, node++, applicable, preferred);
  EXPECT_EQ(0, h);
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
