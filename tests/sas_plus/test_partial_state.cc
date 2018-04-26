#include "sas_plus/partial_state.h"

#include "gtest/gtest.h"

namespace pplanner {

class PartialStateTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    state_.resize(4, 1);
    std::vector<std::pair<int, int> > v{std::make_pair(0, 1)};
    partial_state_1_ = PartialState(v);
    v.push_back(std::make_pair(2, 4));
    partial_state_2_= PartialState(v);
  }

  std::vector<int> state_;
  PartialState partial_state_0_;
  PartialState partial_state_1_;
  PartialState partial_state_2_;
};

TEST_F(PartialStateTest, SizeWorks) {
  EXPECT_EQ(0, partial_state_0_.size());
  EXPECT_EQ(1, partial_state_1_.size());
  EXPECT_EQ(2, partial_state_2_.size());
}

TEST_F(PartialStateTest, AddWorks) {
  partial_state_0_.Add(2, 3);
  EXPECT_EQ(1, partial_state_0_.size());
}

TEST_F(PartialStateTest, IsSubsetWorks) {
  ASSERT_TRUE(partial_state_0_.IsSubset(state_));
  ASSERT_TRUE(partial_state_1_.IsSubset(state_));
  ASSERT_FALSE(partial_state_2_.IsSubset(state_));
}

} // namespace pplanner
