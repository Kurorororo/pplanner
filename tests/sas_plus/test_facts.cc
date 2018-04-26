#include "sas_plus/facts.h"

#include "gtest/gtest.h"

namespace pplanner {

class FactsTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    predicates_0_ = std::vector<std::string>{"a", "b"};
    predicates_1_ = std::vector<std::string>{"c", "b", "d"};
    predicates_2_ = std::vector<std::string>{"a", "e"};

    state_ = std::vector<int>{1, 2, 0};

    facts_1_.AddVariable(predicates_0_);
    facts_1_.AddVariable(predicates_1_);
    facts_1_.AddVariable(predicates_2_);
  }

  std::vector<std::string> predicates_0_;
  std::vector<std::string> predicates_1_;
  std::vector<std::string> predicates_2_;
  std::vector<int> state_;
  Facts facts_0_;
  Facts facts_1_;
};

TEST_F(FactsTest, IsInitialized) {
  EXPECT_EQ(0, facts_0_.size());
  EXPECT_EQ(0, facts_0_.n_variables());
}

TEST_F(FactsTest, AddVariableWorks) {
  int var = facts_0_.AddVariable(predicates_0_);
  EXPECT_EQ(0, var);
  EXPECT_EQ(2, facts_0_.size());
  EXPECT_EQ(1, facts_0_.n_variables());
  var = facts_0_.AddVariable(predicates_1_);
  EXPECT_EQ(1, var);
  EXPECT_EQ(5, facts_0_.size());
  EXPECT_EQ(2, facts_0_.n_variables());
  var = facts_0_.AddVariable(predicates_2_);
  EXPECT_EQ(2, var);
  EXPECT_EQ(7, facts_0_.size());
  EXPECT_EQ(3, facts_0_.n_variables());
}

TEST_F(FactsTest, VarBeginWorks) {
  EXPECT_EQ(0, facts_1_.VarBegin(0));
  EXPECT_EQ(2, facts_1_.VarBegin(1));
  EXPECT_EQ(5, facts_1_.VarBegin(2));
}

TEST_F(FactsTest, VarRangeWorks) {
  EXPECT_EQ(2, facts_1_.VarRange(0));
  EXPECT_EQ(3, facts_1_.VarRange(1));
  EXPECT_EQ(2, facts_1_.VarRange(2));
}

TEST_F(FactsTest, FactWorks) {
  int f = facts_1_.Fact(0, 0);
  EXPECT_EQ(0, f);
  f = facts_1_.Fact(1, 1);
  EXPECT_EQ(3, f);
  f = facts_1_.Fact(2, 1);
  EXPECT_EQ(6, f);
}

TEST_F(FactsTest, PredicateWorks) {
  EXPECT_EQ("a", facts_1_.Predicate(0, 0));
  EXPECT_EQ("b", facts_1_.Predicate(0, 1));
  EXPECT_EQ("c", facts_1_.Predicate(1, 0));
  EXPECT_EQ("b", facts_1_.Predicate(1, 1));
  EXPECT_EQ("d", facts_1_.Predicate(1, 2));
  EXPECT_EQ("a", facts_1_.Predicate(2, 0));
  EXPECT_EQ("e", facts_1_.Predicate(2, 1));
}

TEST_F(FactsTest, StateToFactVectorWorks) {
  std::vector<int> v;
  StateToFactVector(facts_1_, state_, v);
  ASSERT_TRUE(state_.size() == v.size());
  EXPECT_EQ(1, v[0]);
  EXPECT_EQ(4, v[1]);
  EXPECT_EQ(5, v[2]);
}

TEST_F(FactsTest, StateToFactSetWorks) {
  std::vector<bool> s;
  StateToFactSet(facts_1_, state_, s);
  ASSERT_TRUE(facts_1_.size() == s.size());
  ASSERT_FALSE(s[0]);
  ASSERT_TRUE(s[1]);
  ASSERT_FALSE(s[2]);
  ASSERT_FALSE(s[3]);
  ASSERT_TRUE(s[4]);
  ASSERT_TRUE(s[5]);
  ASSERT_FALSE(s[6]);
}

} // namespace pplanner
