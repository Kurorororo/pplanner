#include "sas_plus/partial_state_vector.h"

#include "gtest/gtest.h"

namespace pplanner {

class PartialStateVectorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    v_0_ = std::vector<std::pair<int, int> >{std::make_pair(0, 2),
                                             std::make_pair(1, 1),
                                             std::make_pair(3, 3)};
    v_1_ = std::vector<std::pair<int, int> >{std::make_pair(0, 1),
                                             std::make_pair(2, 0),
                                             std::make_pair(3, 3)};
    p_v_1_.Add(v_0_);
    p_v_1_.Add(v_1_);
  }

  std::vector<std::pair<int, int> > v_0_;
  std::vector<std::pair<int, int> > v_1_;
  PartialStateVector p_v_0_;
  PartialStateVector p_v_1_;
};

TEST_F(PartialStateVectorTest, SizeWorks) {
  EXPECT_EQ(0, p_v_0_.size());
}

TEST_F(PartialStateVectorTest, AddWorks) {
  p_v_0_.Add(v_0_);
  EXPECT_EQ(1, p_v_0_.size());
  p_v_0_.Add(v_1_);
  EXPECT_EQ(2, p_v_0_.size());
}

TEST_F(PartialStateVectorTest, VarRangeWorks) {
  auto iter = p_v_1_.VarsBegin(0);
  EXPECT_EQ(0, *iter);
  ++iter;
  EXPECT_EQ(1, *iter);
  ++iter;
  EXPECT_EQ(3, *iter);
  ++iter;
  auto end = p_v_1_.VarsEnd(0);
  ASSERT_TRUE(iter == end);

  iter = p_v_1_.VarsBegin(1);
  EXPECT_EQ(0, *iter);
  ++iter;
  EXPECT_EQ(2, *iter);
  ++iter;
  EXPECT_EQ(3, *iter);
  ++iter;
  end = p_v_1_.VarsEnd(1);
  ASSERT_TRUE(iter == end);
}

TEST_F(PartialStateVectorTest, ValueRangeWorks) {
  auto iter = p_v_1_.ValuesBegin(0);
  EXPECT_EQ(2, *iter);
  ++iter;
  EXPECT_EQ(1, *iter);
  ++iter;
  EXPECT_EQ(3, *iter);
  ++iter;
  auto end = p_v_1_.ValuesEnd(0);
  ASSERT_TRUE(iter == end);

  iter = p_v_1_.ValuesBegin(1);
  EXPECT_EQ(1, *iter);
  ++iter;
  EXPECT_EQ(0, *iter);
  ++iter;
  EXPECT_EQ(3, *iter);
  ++iter;
  end = p_v_1_.ValuesEnd(1);
  ASSERT_TRUE(iter == end);
}

} // namespace pplanner
