#include "hash/array_hash.h"

#include "gtest/gtest.h"

namespace pplanner {

class ArrayHashTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    hash_ = ArrayHash<int>(3);
  }

  ArrayHash<int> hash_;
};

TEST_F(ArrayHashTest, Works) {
  std::vector<int> state_0{0, 1, 0};
  std::vector<int> state_1{0, 1, 0};
  std::vector<int> state_2{0, 0, 0};
  std::vector<int> state_3{0, 0, 0};

  EXPECT_EQ(hash_(state_0), hash_(state_1));
  EXPECT_EQ(hash_(state_2), hash_(state_3));

  EXPECT_EQ(hash_(state_0.data()), hash_(state_1.data()));
  EXPECT_EQ(hash_(state_2.data()), hash_(state_3.data()));

  EXPECT_EQ(hash_(state_0), hash_(state_1.data()));
  EXPECT_EQ(hash_(state_0.data()), hash_(state_1));
  EXPECT_EQ(hash_(state_2), hash_(state_3.data()));
  EXPECT_EQ(hash_(state_2.data()), hash_(state_3));
}

} // namespace pplanner
