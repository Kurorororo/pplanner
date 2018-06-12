#include "landmark/landmark.h"

#include <memory>
#include <utility>

#include "gtest/gtest.h"

namespace pplanner {

class LandmarkTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    l_0_ = std::make_shared<Landmark>();
    l_1_ = std::make_shared<Landmark>(0, 2);
    l_2_ = std::make_shared<Landmark>(2, 4);
    std::vector<std::pair<int, int> > var_values{
      std::make_pair(0, 2), std::make_pair(2, 1)};
    l_3_ = std::make_shared<Landmark>(var_values);
  }

  std::shared_ptr<Landmark> l_0_;
  std::shared_ptr<Landmark> l_1_;
  std::shared_ptr<Landmark> l_2_;
  std::shared_ptr<Landmark> l_3_;
};

TEST_F(LandmarkTest, EqualWorks) {
  Landmark l(0, 2);
  EXPECT_FALSE(l == *l_0_);
  EXPECT_TRUE(l == *l_1_);
  EXPECT_FALSE(l == *l_2_);
  EXPECT_FALSE(l == *l_3_);
}

TEST_F(LandmarkTest, NotEqualWorks) {
  Landmark l(0, 2);
  EXPECT_TRUE(l != *l_0_);
  EXPECT_FALSE(l != *l_1_);
  EXPECT_TRUE(l != *l_2_);
  EXPECT_TRUE(l != *l_3_);
}

TEST_F(LandmarkTest, SizeWorks) {
  EXPECT_EQ(0, l_0_->size());
  EXPECT_EQ(1, l_1_->size());
  EXPECT_EQ(1, l_2_->size());
  EXPECT_EQ(2, l_3_->size());
}

TEST_F(LandmarkTest, IsEmptyWorks) {
  EXPECT_TRUE(l_0_->IsEmpty());
  EXPECT_FALSE(l_1_->IsEmpty());
  EXPECT_FALSE(l_2_->IsEmpty());
  EXPECT_FALSE(l_3_->IsEmpty());
}

TEST_F(LandmarkTest, IsFactWorks) {
  EXPECT_FALSE(l_0_->IsFact());
  EXPECT_TRUE(l_1_->IsFact());
  EXPECT_TRUE(l_2_->IsFact());
  EXPECT_FALSE(l_3_->IsFact());
}

TEST_F(LandmarkTest, ClearWorks) {
  l_1_->Clear();
  EXPECT_TRUE(l_1_->IsEmpty());
  l_3_->Clear();
  EXPECT_TRUE(l_3_->IsEmpty());
}

TEST_F(LandmarkTest, VarValueWorks) {
  auto p = std::make_pair(0, 2);
  EXPECT_EQ(p, l_1_->VarValue(0));
  EXPECT_NE(p, l_2_->VarValue(0));
  p = std::make_pair(2, 1);
  EXPECT_EQ(p, l_3_->VarValue(1));
  p = std::make_pair(2, 0);
  EXPECT_NE(p, l_3_->VarValue(1));
}

TEST_F(LandmarkTest, VarWorks) {
  EXPECT_EQ(0, l_1_->Var(0));
  EXPECT_NE(1, l_1_->Var(0));
  EXPECT_EQ(2, l_3_->Var(1));
  EXPECT_NE(4, l_3_->Var(1));
}

TEST_F(LandmarkTest, ValueWorks) {
  EXPECT_EQ(2, l_1_->Value(0));
  EXPECT_NE(1, l_1_->Value(0));
  EXPECT_EQ(1, l_3_->Value(1));
  EXPECT_NE(2, l_3_->Value(1));
}

TEST_F(LandmarkTest, AddWorks) {
  l_1_->Add(std::make_pair(0, 2));
  EXPECT_EQ(1, l_1_->size());
  EXPECT_TRUE(l_1_->IsFact());
  l_1_->Add(std::make_pair(2, 1));
  EXPECT_EQ(2, l_1_->size());
  EXPECT_FALSE(l_1_->IsFact());
  EXPECT_TRUE(*l_3_ == *l_1_);
}

TEST_F(LandmarkTest, HashWorks) {
  l_1_->Add(std::make_pair(2, 1));
  EXPECT_TRUE(l_1_->Hash() == l_3_->Hash());
}

TEST_F(LandmarkTest, IsImplicatedWorks) {
  auto p_0 = std::make_pair(0, 1);
  auto p_1 = std::make_pair(0, 2);
  auto p_2 = std::make_pair(2, 4);
  auto p_3 = std::make_pair(2, 1);
  EXPECT_FALSE(l_0_->IsImplicated(p_0));
  EXPECT_FALSE(l_1_->IsImplicated(p_0));
  EXPECT_TRUE(l_1_->IsImplicated(p_1));
  EXPECT_FALSE(l_2_->IsImplicated(p_1));
  EXPECT_TRUE(l_2_->IsImplicated(p_2));
  EXPECT_FALSE(l_3_->IsImplicated(p_2));
  EXPECT_TRUE(l_3_->IsImplicated(p_1));
  EXPECT_TRUE(l_3_->IsImplicated(p_3));
}

TEST_F(LandmarkTest, OverlapWorks) {
  Landmark l(0, 2);
  EXPECT_FALSE(l_1_->Overlap(l));
  EXPECT_FALSE(l_2_->Overlap(l));
  EXPECT_TRUE(l_3_->Overlap(l));
}

} // namespace pplanner
