#include "dtg.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace pplanner {

class DTGTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
   /**
    0-->1-->3
     |    |
     -->2--
    */
    std::vector< std::vector<int> > adjacent_lists(4);
    adjacent_lists[0].push_back(1);
    adjacent_lists[0].push_back(2);
    adjacent_lists[1].push_back(3);
    adjacent_lists[2].push_back(3);
    dtg_ = std::make_shared<DTG>(adjacent_lists);
  }

  std::shared_ptr<DTG> dtg_;
};

TEST_F(DTGTest, IsConnectedWorks) {
  EXPECT_TRUE(dtg_->IsConnected(0, 1));
  EXPECT_TRUE(dtg_->IsConnected(0, 1, 2));
  EXPECT_TRUE(dtg_->IsConnected(0, 1, 3));
  EXPECT_TRUE(dtg_->IsConnected(0, 2));
  EXPECT_TRUE(dtg_->IsConnected(0, 2, 1));
  EXPECT_TRUE(dtg_->IsConnected(0, 2, 3));
  EXPECT_TRUE(dtg_->IsConnected(0, 3));
  EXPECT_TRUE(dtg_->IsConnected(0, 3, 1));
  EXPECT_TRUE(dtg_->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(1, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 3));
  EXPECT_TRUE(dtg_->IsConnected(1, 3));
  EXPECT_TRUE(dtg_->IsConnected(1, 3, 0));
  EXPECT_TRUE(dtg_->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(2, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(2, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 3));
  EXPECT_TRUE(dtg_->IsConnected(2, 3));
  EXPECT_TRUE(dtg_->IsConnected(2, 3, 0));
  EXPECT_TRUE(dtg_->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_->IsConnected(3, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 1));
}

TEST_F(DTGTest, RemoveNodeWorks) {
  dtg_->RemoveNode(1);
  EXPECT_FALSE(dtg_->IsConnected(0, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 1, 2));
  EXPECT_FALSE(dtg_->IsConnected(0, 1, 3));
  EXPECT_TRUE(dtg_->IsConnected(0, 2));
  EXPECT_TRUE(dtg_->IsConnected(0, 2, 1));
  EXPECT_TRUE(dtg_->IsConnected(0, 2, 3));
  EXPECT_TRUE(dtg_->IsConnected(0, 3));
  EXPECT_TRUE(dtg_->IsConnected(0, 3, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(1, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 3, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(2, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(2, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 3));
  EXPECT_TRUE(dtg_->IsConnected(2, 3));
  EXPECT_TRUE(dtg_->IsConnected(2, 3, 0));
  EXPECT_TRUE(dtg_->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_->IsConnected(3, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 1));

  dtg_->RemoveNode(2);
  EXPECT_FALSE(dtg_->IsConnected(0, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 1, 2));
  EXPECT_FALSE(dtg_->IsConnected(0, 1, 3));
  EXPECT_FALSE(dtg_->IsConnected(0, 2));
  EXPECT_FALSE(dtg_->IsConnected(0, 2, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 2, 3));
  EXPECT_FALSE(dtg_->IsConnected(0, 3));
  EXPECT_FALSE(dtg_->IsConnected(0, 3, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(1, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 3, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(2, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(2, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 3));
  EXPECT_FALSE(dtg_->IsConnected(2, 3));
  EXPECT_FALSE(dtg_->IsConnected(2, 3, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_->IsConnected(3, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 1));
}


TEST_F(DTGTest, SoftRemoveNodeWorks) {
  dtg_->SoftRemoveNode(1);
  EXPECT_FALSE(dtg_->IsConnected(0, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 1, 2));
  EXPECT_FALSE(dtg_->IsConnected(0, 1, 3));
  EXPECT_TRUE(dtg_->IsConnected(0, 2));
  EXPECT_TRUE(dtg_->IsConnected(0, 2, 1));
  EXPECT_TRUE(dtg_->IsConnected(0, 2, 3));
  EXPECT_TRUE(dtg_->IsConnected(0, 3));
  EXPECT_TRUE(dtg_->IsConnected(0, 3, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(1, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 3, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(2, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(2, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 3));
  EXPECT_TRUE(dtg_->IsConnected(2, 3));
  EXPECT_TRUE(dtg_->IsConnected(2, 3, 0));
  EXPECT_TRUE(dtg_->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_->IsConnected(3, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 1));

  dtg_->SoftRemoveNode(2);
  EXPECT_FALSE(dtg_->IsConnected(0, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 1, 2));
  EXPECT_FALSE(dtg_->IsConnected(0, 1, 3));
  EXPECT_FALSE(dtg_->IsConnected(0, 2));
  EXPECT_FALSE(dtg_->IsConnected(0, 2, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 2, 3));
  EXPECT_FALSE(dtg_->IsConnected(0, 3));
  EXPECT_FALSE(dtg_->IsConnected(0, 3, 1));
  EXPECT_FALSE(dtg_->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(1, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 2));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 2, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 3));
  EXPECT_FALSE(dtg_->IsConnected(1, 3, 0));
  EXPECT_FALSE(dtg_->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_->IsConnected(2, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_->IsConnected(2, 1));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 1, 3));
  EXPECT_FALSE(dtg_->IsConnected(2, 3));
  EXPECT_FALSE(dtg_->IsConnected(2, 3, 0));
  EXPECT_FALSE(dtg_->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_->IsConnected(3, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 1));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_->IsConnected(3, 2, 1));
}

} // namespace pplanner