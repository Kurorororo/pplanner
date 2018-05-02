#include "open_lists/fifo_open_list_impl.h"

#include "gtest/gtest.h"

namespace pplanner {

class FIFOOpenListImplTest: public ::testing::Test {
 protected:
  virtual void SetUp() {}

  FIFOOpenListImpl list_;
};

TEST_F(FIFOOpenListImplTest, IsEmptyWorks) {
  EXPECT_TRUE(list_.IsEmpty());
}

TEST_F(FIFOOpenListImplTest, PushWorks) {
  int node = 0;
  std::vector<int> values{0, 1, 2};
  list_.Push(values, node);
  EXPECT_FALSE(list_.IsEmpty());
}

TEST_F(FIFOOpenListImplTest, PopWorks) {
  int node = 0;
  std::vector<int> values{0, 1, 2};
  list_.Push(values, node);
  node = 1;
  values[0] = 1;
  list_.Push(values, node);
  values[1] = 0;
  node = 2;
  list_.Push(values, node);
  values[0] = 0;
  node = 3;
  list_.Push(values, node);
  values[1] = 1;
  node = 4;
  list_.Push(values, node);
  node = 5;
  list_.Push(values, node);

  node = list_.Pop();
  EXPECT_EQ(3, node);
  node = list_.Pop();
  EXPECT_EQ(0, node);
  node = list_.Pop();
  EXPECT_EQ(4, node);
  node = list_.Pop();
  EXPECT_EQ(5, node);
  node = list_.Pop();
  EXPECT_EQ(2, node);
  node = list_.Pop();
  EXPECT_EQ(1, node);
}

} // namespace pplanner
