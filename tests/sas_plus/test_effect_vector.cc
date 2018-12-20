#include "sas_plus/effect_vector.h"

#include "gtest/gtest.h"

namespace pplanner {

class EffectVectorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    v_0_ = std::vector<std::pair<int, int> >{std::make_pair(0, 2),
                                             std::make_pair(1, 1),
                                             std::make_pair(3, 3)};
    v_1_ = std::vector<std::pair<int, int> >{std::make_pair(0, 1),
                                             std::make_pair(2, 0),
                                             std::make_pair(3, 3)};
    v_2_ = std::vector<std::pair<int, int> >{std::make_pair(0, 1),
                                             std::make_pair(3, 3)};

    e_v_1_.Add(v_0_);
    e_v_1_.AddConditionalEffect(e_c_, c_e_);
    e_v_1_.Add(v_1_);
    e_v_1_.AddConditionalEffect(e_c_, c_e_);
    e_v_1_.Add(v_2_);

    e_c_ = std::vector<std::vector<std::pair<int, int> > >{
      {std::make_pair(1, 1)}, {std::make_pair(0, 0), std::make_pair(3, 2)}};
    c_e_ = std::vector<std::pair<int, int> >{
      std::make_pair(1, 0), std::make_pair(2, 2)};

    e_v_1_.AddConditionalEffect(e_c_, c_e_);
  }

  std::vector<std::pair<int, int> > v_0_;
  std::vector<std::pair<int, int> > v_1_;
  std::vector<std::pair<int, int> > v_2_;
  std::vector<std::vector<std::pair<int, int> > > e_c_;
  std::vector<std::pair<int, int> > c_e_;
  EffectVector e_v_0_;
  EffectVector e_v_1_;
};

TEST_F(EffectVectorTest, ApplyWorks) {
  std::vector<int> state{0, 2, 2, 1};
  std::vector<int> expect{2, 1, 2, 3};
  e_v_1_.Apply(0, state);
  EXPECT_EQ(expect, state);

  expect = std::vector<int>{1, 1, 0, 3};
  e_v_1_.Apply(1, state);
  EXPECT_EQ(expect, state);

  // no conditional effect
  state = std::vector<int>{2, 2, 0, 0};
  expect = std::vector<int>{1, 2, 0, 3};
  e_v_1_.Apply(2, state);
  EXPECT_EQ(expect, state);

  // fire one conditoonal effect
  state = std::vector<int>{2, 1, 0, 0};
  expect = std::vector<int>{1, 0, 0, 3};
  e_v_1_.Apply(2, state);
  EXPECT_EQ(expect, state);

  // satisfy partial condition
  state = std::vector<int>{0, 2, 0, 0};
  expect = std::vector<int>{1, 2, 0, 3};
  e_v_1_.Apply(2, state);
  EXPECT_EQ(expect, state);

  // satisfy partial condition
  state = std::vector<int>{2, 2, 0, 2};
  expect = std::vector<int>{1, 2, 0, 3};
  e_v_1_.Apply(2, state);
  EXPECT_EQ(expect, state);

  // satisfy condition
  state = std::vector<int>{0, 2, 0, 2};
  expect = std::vector<int>{1, 2, 2, 3};
  e_v_1_.Apply(2, state);
  EXPECT_EQ(expect, state);

  // fire all conditional effects
  state = std::vector<int>{0, 1, 0, 2};
  expect = std::vector<int>{1, 0, 2, 3};
  e_v_1_.Apply(2, state);
  EXPECT_EQ(expect, state);
}

TEST_F(EffectVectorTest, HasConditionalEffectsWorks) {
  EXPECT_FALSE(e_v_1_.HasConditionalEffects(0));
  EXPECT_FALSE(e_v_1_.HasConditionalEffects(1));
  EXPECT_TRUE(e_v_1_.HasConditionalEffects(2));
}

TEST_F(EffectVectorTest, UseConditionalTest) {
  EXPECT_FALSE(e_v_0_.use_conditional());
  e_v_0_.Add(v_0_);
  EXPECT_FALSE(e_v_0_.use_conditional());
  e_v_0_.Add(v_1_);
  EXPECT_FALSE(e_v_0_.use_conditional());
  e_v_0_.Add(v_2_);
  EXPECT_FALSE(e_v_0_.use_conditional());
  e_v_0_.AddConditionalEffect(e_c_, c_e_);
  EXPECT_TRUE(e_v_0_.use_conditional());
}

} // namespace pplanner
