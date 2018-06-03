#include "hash/pair_hash.h"

#include <utility>

#include "gtest/gtest.h"

namespace pplanner {

class PairHashTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  PairHash<int, int> hash_;
};

TEST_F(PairHashTest, Works) {
  std::pair<int, int> p1(2, 3);
  std::pair<int, int> p2(2, 3);

  EXPECT_EQ(hash_(p1), hash_(p2));
}

} // namespace pplanner
