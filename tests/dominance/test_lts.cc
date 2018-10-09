#include "dominance/lts.h"

#include <memory>
#include <vector>
#include <iostream>

#include "gtest/gtest.h"

namespace pplanner {

class AtomicLTSTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    int initial = 0;
    int goal = 3;

    std::vector<int> label_from{0, 1, 2, 1, 4, 5, 4, -1, 4};
    std::vector<int> label_to{1, 2, 3, 4, 5, 3, 6, 1, -1};
    std::vector<bool> is_tau_label{
      true, false, false, true, true, true, false, false, false
    };
    std::vector<std::vector<int> > to(7);
    std::vector<std::vector<std::vector<int> > > labels(
        7, std::vector<std::vector<int> >(7, std::vector<int>{-1}));

    for (int s=0; s<7; ++s)
      to[s].push_back(s);

    for (int i=0; i<9; ++i) {
      int s = label_from[i];
      int t = label_to[i];

      if (s == -1) {
        for (int j=0; j<7; ++j) {
          labels[j][t].push_back(i);
          to[j].push_back(t);
        }
      } else {
        if (t == -1) t = s;
        labels[s][t].push_back(i);
        to[s].push_back(t);
      }
    }

    lts_ = std::make_shared<AtomicLTS>(
        initial, goal, label_from, label_to, is_tau_label, to, labels);
  }

 std::shared_ptr<AtomicLTS> lts_;
};

TEST_F(AtomicLTSTest, NStatesWorks) {
 EXPECT_EQ(7, lts_->n_states());
}

TEST_F(AtomicLTSTest, NLabelsWorks) {
 EXPECT_EQ(9, lts_->n_labels());
}

TEST_F(AtomicLTSTest, InitialWorks) {
  EXPECT_EQ(0, lts_->initial());
}

TEST_F(AtomicLTSTest, GoalWorks) {
  EXPECT_EQ(3, lts_->goal());
}

TEST_F(AtomicLTSTest, LabelFromWorks) {
  EXPECT_EQ(-1, lts_->LabelFrom(-1));
  EXPECT_EQ(0, lts_->LabelFrom(0));
  EXPECT_EQ(1, lts_->LabelFrom(1));
  EXPECT_EQ(2, lts_->LabelFrom(2));
  EXPECT_EQ(1, lts_->LabelFrom(3));
  EXPECT_EQ(4, lts_->LabelFrom(4));
  EXPECT_EQ(5, lts_->LabelFrom(5));
  EXPECT_EQ(4, lts_->LabelFrom(6));
  EXPECT_EQ(-1, lts_->LabelFrom(7));
  EXPECT_EQ(4, lts_->LabelFrom(8));
}

TEST_F(AtomicLTSTest, LabelToWorks) {
  EXPECT_EQ(-1, lts_->LabelTo(-1));
  EXPECT_EQ(1, lts_->LabelTo(0));
  EXPECT_EQ(2, lts_->LabelTo(1));
  EXPECT_EQ(3, lts_->LabelTo(2));
  EXPECT_EQ(4, lts_->LabelTo(3));
  EXPECT_EQ(5, lts_->LabelTo(4));
  EXPECT_EQ(3, lts_->LabelTo(5));
  EXPECT_EQ(6, lts_->LabelTo(6));
  EXPECT_EQ(1, lts_->LabelTo(7));
  EXPECT_EQ(-1, lts_->LabelTo(8));
}

TEST_F(AtomicLTSTest, TransitionCostWorks) {
  for (int s=0; s<7; ++s) {
    for (int t=0; t<7; ++t) {
      if (s == t)
        EXPECT_EQ(0, lts_->TransitionCost(s, t));
      else
        EXPECT_EQ(1, lts_->TransitionCost(s, t));
    }
  }
}

TEST_F(AtomicLTSTest, TauTransitionCostWorks) {
  for (int s=0; s<7; ++s) {
    for (int t=0; t<7; ++t) {
      if ((s == 0 && t == 1)
          || (s == 1 && t == 4)
          || (s == 4 && t == 5)
          || (s == 5 && t == 3))
        EXPECT_EQ(1, lts_->TauTransitionCost(s, t));
      else
        EXPECT_EQ(AtomicLTS::kInfinity, lts_->TauTransitionCost(s, t));
    }
  }
}

TEST_F(AtomicLTSTest, IsTauLabelWorks) {
  EXPECT_FALSE(lts_->IsTauLabel(-1));
  EXPECT_TRUE(lts_->IsTauLabel(0));
  EXPECT_FALSE(lts_->IsTauLabel(1));
  EXPECT_FALSE(lts_->IsTauLabel(2));
  EXPECT_TRUE(lts_->IsTauLabel(3));
  EXPECT_TRUE(lts_->IsTauLabel(4));
  EXPECT_TRUE(lts_->IsTauLabel(5));
  EXPECT_FALSE(lts_->IsTauLabel(6));
  EXPECT_FALSE(lts_->IsTauLabel(7));
  EXPECT_FALSE(lts_->IsTauLabel(8));
}

TEST_F(AtomicLTSTest, ShortestPathCostWorks) {
  EXPECT_EQ(0, lts_->ShortestPathCost(0, 0));
  EXPECT_EQ(0, lts_->ShortestPathCost(1, 1));
  EXPECT_EQ(0, lts_->ShortestPathCost(2, 2));
  EXPECT_EQ(0, lts_->ShortestPathCost(3, 3));
  EXPECT_EQ(0, lts_->ShortestPathCost(4, 4));
  EXPECT_EQ(0, lts_->ShortestPathCost(5, 5));
  EXPECT_EQ(0, lts_->ShortestPathCost(6, 6));

  EXPECT_EQ(1, lts_->ShortestPathCost(0, 1));
  EXPECT_EQ(1, lts_->ShortestPathCost(2, 1));
  EXPECT_EQ(1, lts_->ShortestPathCost(3, 1));
  EXPECT_EQ(1, lts_->ShortestPathCost(4, 1));
  EXPECT_EQ(1, lts_->ShortestPathCost(5, 1));
  EXPECT_EQ(1, lts_->ShortestPathCost(6, 1));

  EXPECT_EQ(2, lts_->ShortestPathCost(0, 2));
  EXPECT_EQ(3, lts_->ShortestPathCost(0, 3));
  EXPECT_EQ(AtomicLTS::kInfinity, lts_->ShortestPathCost(0, 2, true));
  EXPECT_EQ(4, lts_->ShortestPathCost(0, 3, true));

  EXPECT_EQ(AtomicLTS::kInfinity, lts_->ShortestPathCost(2, 0));
  EXPECT_EQ(AtomicLTS::kInfinity, lts_->ShortestPathCost(3, 0));
  EXPECT_EQ(AtomicLTS::kInfinity, lts_->ShortestPathCost(4, 0));
  EXPECT_EQ(AtomicLTS::kInfinity, lts_->ShortestPathCost(5, 0));
  EXPECT_EQ(AtomicLTS::kInfinity, lts_->ShortestPathCost(6, 0));

  EXPECT_EQ(2, lts_->ShortestPathCost(1, 6));
  EXPECT_EQ(AtomicLTS::kInfinity, lts_->ShortestPathCost(1, 6, true));
}

} // namespace pplanner
