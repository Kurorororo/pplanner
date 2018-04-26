#include "sas_plus/mutex_groups.h"

#include "gtest/gtest.h"

namespace pplanner {

class MutexGroupsTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    group_0_ = std::vector<int>{1, 2, 3};
    group_1_ = std::vector<int>{1, 4, 6};
    mutex_groups_1_.AddGroup(group_0_);
    mutex_groups_1_.AddGroup(group_1_);
  }

  std::vector<int> group_0_;
  std::vector<int> group_1_;
  MutexGroups mutex_groups_0_;
  MutexGroups mutex_groups_1_;
};

TEST_F(MutexGroupsTest, SizeWorks) {
  EXPECT_EQ(0, mutex_groups_0_.size());
}

TEST_F(MutexGroupsTest, AddGroupWorks) {
  mutex_groups_0_.AddGroup(group_0_);
  EXPECT_EQ(1, mutex_groups_0_.size());
  mutex_groups_0_.AddGroup(group_1_);
  EXPECT_EQ(2, mutex_groups_0_.size());
}

TEST_F(MutexGroupsTest, IsMutexWorks) {
  int f = 1;
  int g = 2;
  ASSERT_FALSE(mutex_groups_0_.IsMutex(f, g));
  ASSERT_TRUE(mutex_groups_1_.IsMutex(f, g));
  int h = 4;
  ASSERT_TRUE(mutex_groups_1_.IsMutex(f, h));
  ASSERT_FALSE(mutex_groups_1_.IsMutex(g, h));
}

} // namespace pplanner
