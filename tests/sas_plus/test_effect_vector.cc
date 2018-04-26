#include "sas_plus/effect_vector.h"

#include "gtest/gtest.h"

namespace pplanner {

class EffectVectorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    v_0_ = std::vector<std::pair<int, int> >{std::make_pair(0, 2),
                                             std::make_pair(1, 1),
                                             std::make_pair(3, 3)};
    v_1_ = std::vector<std::pair<int, int> >{std::make_pair(0, 1),
                                             std::make_pair(2, 0),
                                             std::make_pair(3, 3)};
    e_v_1_.Add(v_0_);
    e_v_1_.Add(v_1_);
  }

  std::vector<std::pair<int, int> > v_0_;
  std::vector<std::pair<int, int> > v_1_;
  EffectVector e_v_0_;
  EffectVector e_v_1_;
};

TEST_F(EffectVectorTest, SizeWorks) {
  EXPECT_EQ(0, e_v_0_.size());
}

TEST_F(EffectVectorTest, AddWorks) {
  e_v_0_.Add(v_0_);
  EXPECT_EQ(1, e_v_0_.size());
  e_v_0_.Add(v_1_);
  EXPECT_EQ(2, e_v_0_.size());
}

TEST_F(EffectVectorTest, VarRangeWorks) {
  auto iter = e_v_1_.VarsBegin(0);
  EXPECT_EQ(0, *iter);
  ++iter;
  EXPECT_EQ(1, *iter);
  ++iter;
  EXPECT_EQ(3, *iter);
  ++iter;
  auto end = e_v_1_.VarsEnd(0);
  ASSERT_TRUE(iter == end);

  iter = e_v_1_.VarsBegin(1);
  EXPECT_EQ(0, *iter);
  ++iter;
  EXPECT_EQ(2, *iter);
  ++iter;
  EXPECT_EQ(3, *iter);
  ++iter;
  end = e_v_1_.VarsEnd(1);
  ASSERT_TRUE(iter == end);
}

TEST_F(EffectVectorTest, ValueRangeWorks) {
  auto iter = e_v_1_.ValuesBegin(0);
  EXPECT_EQ(2, *iter);
  ++iter;
  EXPECT_EQ(1, *iter);
  ++iter;
  EXPECT_EQ(3, *iter);
  ++iter;
  auto end = e_v_1_.ValuesEnd(0);
  ASSERT_TRUE(iter == end);

  iter = e_v_1_.ValuesBegin(1);
  EXPECT_EQ(1, *iter);
  ++iter;
  EXPECT_EQ(0, *iter);
  ++iter;
  EXPECT_EQ(3, *iter);
  ++iter;
  end = e_v_1_.ValuesEnd(1);
  ASSERT_TRUE(iter == end);
}

TEST_F(EffectVectorTest, ApplyWorks) {
  std::vector<int> state{0, 2, 2, 1};
  e_v_1_.Apply(0, state);
  EXPECT_EQ(2, state[0]);
  EXPECT_EQ(1, state[1]);
  EXPECT_EQ(2, state[2]);
  EXPECT_EQ(3, state[3]);
  e_v_1_.Apply(1, state);
  EXPECT_EQ(1, state[0]);
  EXPECT_EQ(1, state[1]);
  EXPECT_EQ(0, state[2]);
  EXPECT_EQ(3, state[3]);
}

} // namespace pplanner
