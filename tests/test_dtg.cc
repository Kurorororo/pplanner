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
    std::vector<std::vector<int> > adjacent_matrix_1(4, std::vector<int>(4, 0));
    adjacent_matrix_1[0][1] = 1;
    adjacent_matrix_1[0][2] = 1;
    adjacent_matrix_1[1][3] = 1;
    adjacent_matrix_1[2][3] = 1;
    dtg_1 = std::make_shared<DTG>(adjacent_matrix_1);

    // logistics domain of 2 cities with 10/6 locations.
    std::vector<std::vector<int> > adjacent_matrix_2(
        16, std::vector<int>(16, 0));

    for (int i=0; i<10; ++i)
      for (int j=0; j<10; ++j)
        if (i != j) adjacent_matrix_2[i][j] = 1;

    for (int i=10; i<16; ++i)
      for (int j=10; j<16; ++j)
        if (i != j) adjacent_matrix_2[i][j] = 1;

    adjacent_matrix_2[9][10] = 1;
    adjacent_matrix_2[10][9] = 1;

    dtg_2 = std::make_shared<DTG>(adjacent_matrix_2);
  }

  std::shared_ptr<DTG> dtg_1;
  std::shared_ptr<DTG> dtg_2;
};

TEST_F(DTGTest, IsConnectedWorks) {
  EXPECT_TRUE(dtg_1->IsConnected(0, 1));
  EXPECT_TRUE(dtg_1->IsConnected(0, 1, 2));
  EXPECT_TRUE(dtg_1->IsConnected(0, 1, 3));
  EXPECT_TRUE(dtg_1->IsConnected(0, 2));
  EXPECT_TRUE(dtg_1->IsConnected(0, 2, 1));
  EXPECT_TRUE(dtg_1->IsConnected(0, 2, 3));
  EXPECT_TRUE(dtg_1->IsConnected(0, 3));
  EXPECT_TRUE(dtg_1->IsConnected(0, 3, 1));
  EXPECT_TRUE(dtg_1->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 3));
  EXPECT_TRUE(dtg_1->IsConnected(1, 3));
  EXPECT_TRUE(dtg_1->IsConnected(1, 3, 0));
  EXPECT_TRUE(dtg_1->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 3));
  EXPECT_TRUE(dtg_1->IsConnected(2, 3));
  EXPECT_TRUE(dtg_1->IsConnected(2, 3, 0));
  EXPECT_TRUE(dtg_1->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_1->IsConnected(3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 1));
}

TEST_F(DTGTest, RemoveNodeWorks) {
  dtg_1->RemoveNode(1);
  EXPECT_FALSE(dtg_1->IsConnected(0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(0, 1, 3));
  EXPECT_TRUE(dtg_1->IsConnected(0, 2));
  EXPECT_TRUE(dtg_1->IsConnected(0, 2, 1));
  EXPECT_TRUE(dtg_1->IsConnected(0, 2, 3));
  EXPECT_TRUE(dtg_1->IsConnected(0, 3));
  EXPECT_TRUE(dtg_1->IsConnected(0, 3, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 3));
  EXPECT_TRUE(dtg_1->IsConnected(2, 3));
  EXPECT_TRUE(dtg_1->IsConnected(2, 3, 0));
  EXPECT_TRUE(dtg_1->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_1->IsConnected(3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 1));

  dtg_1->RemoveNode(2);
  EXPECT_FALSE(dtg_1->IsConnected(0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(0, 1, 3));
  EXPECT_FALSE(dtg_1->IsConnected(0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(0, 2, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 2, 3));
  EXPECT_FALSE(dtg_1->IsConnected(0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(0, 3, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 3));
  EXPECT_FALSE(dtg_1->IsConnected(2, 3));
  EXPECT_FALSE(dtg_1->IsConnected(2, 3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_1->IsConnected(3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 1));
}


TEST_F(DTGTest, SoftRemoveNodeWorks) {
  dtg_1->SoftRemoveNode(1);
  EXPECT_FALSE(dtg_1->IsConnected(0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(0, 1, 3));
  EXPECT_TRUE(dtg_1->IsConnected(0, 2));
  EXPECT_TRUE(dtg_1->IsConnected(0, 2, 1));
  EXPECT_TRUE(dtg_1->IsConnected(0, 2, 3));
  EXPECT_TRUE(dtg_1->IsConnected(0, 3));
  EXPECT_TRUE(dtg_1->IsConnected(0, 3, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 3));
  EXPECT_TRUE(dtg_1->IsConnected(2, 3));
  EXPECT_TRUE(dtg_1->IsConnected(2, 3, 0));
  EXPECT_TRUE(dtg_1->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_1->IsConnected(3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 1));

  dtg_1->SoftRemoveNode(2);
  EXPECT_FALSE(dtg_1->IsConnected(0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(0, 1, 3));
  EXPECT_FALSE(dtg_1->IsConnected(0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(0, 2, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 2, 3));
  EXPECT_FALSE(dtg_1->IsConnected(0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(0, 3, 1));
  EXPECT_FALSE(dtg_1->IsConnected(0, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 2, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(1, 3, 2));

  EXPECT_FALSE(dtg_1->IsConnected(2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 0, 3));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 1, 3));
  EXPECT_FALSE(dtg_1->IsConnected(2, 3));
  EXPECT_FALSE(dtg_1->IsConnected(2, 3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(2, 3, 1));

  EXPECT_FALSE(dtg_1->IsConnected(3, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 0, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 1, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 0));
  EXPECT_FALSE(dtg_1->IsConnected(3, 2, 1));
}

TEST_F(DTGTest, SparsestCutWorks) {
  std::vector<int> cut;
  dtg_2->SparsestCut(cut);

  EXPECT_EQ(cut[0], cut[1]);
  EXPECT_EQ(cut[1], cut[2]);
  EXPECT_EQ(cut[2], cut[3]);
  EXPECT_EQ(cut[3], cut[4]);
  EXPECT_EQ(cut[4], cut[5]);
  EXPECT_EQ(cut[5], cut[6]);
  EXPECT_EQ(cut[6], cut[7]);
  EXPECT_EQ(cut[7], cut[8]);
  EXPECT_EQ(cut[8], cut[9]);
  EXPECT_NE(cut[9], cut[10]);
  EXPECT_EQ(cut[10], cut[11]);
  EXPECT_EQ(cut[11], cut[12]);
  EXPECT_EQ(cut[12], cut[13]);
  EXPECT_EQ(cut[13], cut[14]);
  EXPECT_EQ(cut[14], cut[15]);
}

} // namespace pplanner
