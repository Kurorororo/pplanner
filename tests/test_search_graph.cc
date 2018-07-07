#include "search_graph.h"

#include <cstdio>

#include <queue>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

std::queue<std::string> RandomSASPlusLines(std::vector<int> &ranges);

class SearchGraphTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    sas_0_ = std::make_shared<SASPlus>();
    sas_0_->InitFromLines(lines);
    graph_0_ = std::make_shared<SearchGraph>(sas_0_);

    state_0_ = std::vector<int>{1, 0, 2};

    std::vector<int> ranges;
    lines = RandomSASPlusLines(ranges);
    sas_1_ = std::make_shared<SASPlus>();
    sas_1_->InitFromLines(lines);
    graph_1_ = std::make_shared<SearchGraph>(sas_1_);

    for (auto range : ranges)
      state_1_.push_back(rand() % range);

    state_1_[0] = 1;
    state_1_[1] = 0;
    state_1_[2] = 2;

    graph_2_ = std::make_shared<SearchGraph>(sas_0_, 2);
  }

  std::shared_ptr<SearchGraph> graph_0_;
  std::shared_ptr<SearchGraph> graph_1_;
  std::shared_ptr<SearchGraph> graph_2_;
  std::shared_ptr<SASPlus> sas_0_;
  std::shared_ptr<SASPlus> sas_1_;
  std::vector<int> state_0_;
  std::vector<int> state_1_;
};

TEST_F(SearchGraphTest, SizeWorks) {
  EXPECT_EQ(0, graph_0_->size());
  EXPECT_EQ(0, graph_1_->size());
}

TEST_F(SearchGraphTest, NodeSizeWorks) {
  EXPECT_EQ(2 * sizeof(int) + sizeof(uint32_t) + sizeof(uint32_t),
            graph_0_->node_size());
}

TEST_F(SearchGraphTest, GenerateNodeWorks) {
  int node = graph_0_->GenerateNode(-1, -1, state_0_);
  EXPECT_EQ(0, node);
  EXPECT_EQ(1, graph_0_->size());
  sas_0_->ApplyEffect(4, state_0_);
  node = graph_0_->GenerateNode(4, node, state_0_);
  EXPECT_EQ(1, node);
  EXPECT_EQ(2, graph_0_->size());

  node = graph_1_->GenerateNode(-1, -1, state_1_);
  EXPECT_EQ(0, node);
  EXPECT_EQ(1, graph_1_->size());
  sas_1_->ApplyEffect(4, state_1_);
  node = graph_1_->GenerateNode(4, node, state_1_);
  EXPECT_EQ(1, node);
  EXPECT_EQ(2, graph_1_->size());
}

TEST_F(SearchGraphTest, GenerateNodeWithParentWorks) {
  int node = graph_0_->GenerateNode(-1, -1, state_0_);
  EXPECT_EQ(0, node);
  EXPECT_EQ(1, graph_0_->size());
  std::vector<int> tmp_state_0_(state_0_);
  sas_0_->ApplyEffect(4, state_0_);
  node = graph_0_->GenerateNode(4, node, tmp_state_0_, state_0_);
  EXPECT_EQ(1, node);
  EXPECT_EQ(2, graph_0_->size());

  node = graph_1_->GenerateNode(-1, -1, state_1_);
  EXPECT_EQ(0, node);
  EXPECT_EQ(1, graph_1_->size());
  std::vector<int> tmp_state_1_(state_1_);
  sas_1_->ApplyEffect(4, state_1_);
  node = graph_1_->GenerateNode(4, node, tmp_state_1_, state_1_);
  EXPECT_EQ(1, node);
  EXPECT_EQ(2, graph_1_->size());
}

TEST_F(SearchGraphTest, GenerateNodeIfNotClosedWorks) {
  int i = graph_0_->GenerateNodeIfNotClosed(-1, -1, state_0_);
  EXPECT_EQ(0, i);
  graph_0_->Close(i);
  i = graph_0_->GenerateNodeIfNotClosed(-1, -1, state_0_);
  EXPECT_EQ(-1, i);
  std::vector<int> tmp_state_0(state_0_);
  tmp_state_0[1] = 1;
  i = graph_0_->GenerateNodeIfNotClosed(-1, -1, tmp_state_0);
  EXPECT_EQ(1, i);
  graph_0_->Close(i);
  i = graph_0_->GenerateNodeIfNotClosed(-1, -1, tmp_state_0);
  EXPECT_EQ(-1, i);

  i = graph_1_->GenerateNodeIfNotClosed(-1, -1, state_1_);
  EXPECT_EQ(0, i);
  graph_1_->Close(i);
  i = graph_1_->GenerateNodeIfNotClosed(-1, -1, state_1_);
  EXPECT_EQ(-1, i);
  std::vector<int> tmp_state_1(state_1_);
  tmp_state_1[1] = 1;
  i = graph_1_->GenerateNodeIfNotClosed(-1, -1, tmp_state_1);
  EXPECT_EQ(1, i);
  graph_1_->Close(i);
  i = graph_1_->GenerateNodeIfNotClosed(-1, -1, tmp_state_1);
  EXPECT_EQ(-1, i);
}

TEST_F(SearchGraphTest, ClosedSizeWorks) {
  EXPECT_EQ((1 << 22) * sizeof(int), graph_0_->closed_size());
  EXPECT_EQ((1 << 22) * sizeof(int), graph_1_->closed_size());
  EXPECT_EQ((1 << 2) * sizeof(int), graph_2_->closed_size());
}

TEST_F(SearchGraphTest, ActionWorks) {
  int node = graph_0_->GenerateNode(-1, -1, state_0_);
  EXPECT_EQ(-1, graph_0_->Action(node));
  sas_0_->ApplyEffect(4, state_0_);
  node = graph_0_->GenerateNode(4, node, state_0_);
  EXPECT_EQ(4, graph_0_->Action(node));
}

TEST_F(SearchGraphTest, ParentWorks) {
  int node = graph_0_->GenerateNode(-1, -1, state_0_);
  EXPECT_EQ(-1, graph_0_->Parent(node));
  sas_0_->ApplyEffect(4, state_0_);
  node = graph_0_->GenerateNode(4, node, state_0_);
  EXPECT_EQ(0, graph_0_->Parent(node));
}

TEST_F(SearchGraphTest, StateWorks) {
  int i = graph_0_->GenerateNode(-1, -1, state_0_);
  std::vector<int> tmp(state_0_.size());
  graph_0_->State(i, tmp);
  EXPECT_EQ(state_0_, tmp);
  i = graph_0_->GenerateNode(-1, -1, state_0_);
  graph_0_->State(i, tmp);
  EXPECT_EQ(state_0_, tmp);

  i = graph_1_->GenerateNode(-1, -1, state_1_);
  tmp.resize(state_1_.size());
  graph_1_->State(i, tmp);
  EXPECT_EQ(state_1_, tmp);
  i = graph_1_->GenerateNode(-1, -1, state_1_);
  graph_1_->State(i, tmp);
  EXPECT_EQ(state_1_, tmp);
}

TEST_F(SearchGraphTest, CloseWorks) {
  int i = graph_2_->GenerateNode(-1, -1 ,state_1_);
  graph_2_->Close(i);
  i = graph_2_->GenerateNode(-1, -1, state_1_);
  graph_2_->Close(i);
  i = graph_2_->GenerateNode(-1, -1, state_1_);
  graph_2_->Close(i);
  i = graph_2_->GenerateNode(-1, -1, state_1_);
  graph_2_->Close(i);
  EXPECT_GE(graph_2_->closed_size(), 4);
}

TEST_F(SearchGraphTest, GetClosedWorks) {
  int i = graph_0_->GenerateNode(-1, -1, state_0_);
  EXPECT_EQ(-1, graph_0_->GetClosed(i));
  graph_0_->Close(i);
  EXPECT_EQ(i, graph_0_->GetClosed(i));
  std::vector<int> tmp_state_0(state_0_);
  tmp_state_0[1] = 1;
  i = graph_0_->GenerateNode(-1, -1, tmp_state_0);
  EXPECT_EQ(-1, graph_0_->GetClosed(i));
  graph_0_->Close(i);
  EXPECT_EQ(i, graph_0_->GetClosed(i));

  i = graph_1_->GenerateNode(-1, -1, state_1_);
  EXPECT_EQ(-1, graph_1_->GetClosed(i));
  graph_1_->Close(i);
  EXPECT_EQ(i, graph_1_->GetClosed(i));
  std::vector<int> tmp_state_1(state_1_);
  tmp_state_1[1] = 1;
  i = graph_1_->GenerateNode(-1, -1, tmp_state_1);
  EXPECT_EQ(-1, graph_1_->GetClosed(i));
  graph_1_->Close(i);
  EXPECT_EQ(i, graph_1_->GetClosed(i));

  i = graph_2_->GenerateNode(-1, -1, state_0_);
  EXPECT_EQ(-1, graph_2_->GetClosed(i));
  graph_2_->Close(i);
  EXPECT_EQ(i, graph_2_->GetClosed(i));
  tmp_state_0 = state_0_;
  tmp_state_0[1] = 1;
  i = graph_2_->GenerateNode(-1, -1, tmp_state_0);
  EXPECT_EQ(-1, graph_2_->GetClosed(i));
  graph_2_->Close(i);
  EXPECT_EQ(i, graph_2_->GetClosed(i));
  tmp_state_0[2] = 1;
  i = graph_2_->GenerateNode(-1, -1, tmp_state_0);
  EXPECT_EQ(-1, graph_2_->GetClosed(i));
  graph_2_->Close(i);
  EXPECT_EQ(0, graph_2_->GetClosed(0));
  EXPECT_EQ(i, graph_2_->GetClosed(i));
  EXPECT_EQ(1, graph_2_->GetClosed(1));
  tmp_state_0 = state_0_;
  tmp_state_0[1] = 1;
  tmp_state_0[2] = 0;
  i = graph_2_->GenerateNode(-1, -1, tmp_state_0);
  EXPECT_EQ(-1, graph_2_->GetClosed(i));
  graph_2_->Close(i);
  EXPECT_EQ(i, graph_2_->GetClosed(i));
}

TEST_F(SearchGraphTest, CloseIfNotWorks) {
  int i = graph_0_->GenerateNode(-1, -1, state_0_);
  std::vector<int> tmp_state_0(state_0_.size());
  EXPECT_TRUE(graph_0_->CloseIfNot(i));
  EXPECT_FALSE(graph_0_->CloseIfNot(i));
  state_0_[1] = 1;
  i = graph_0_->GenerateNode(-1, -1, state_0_);
  EXPECT_TRUE(graph_0_->CloseIfNot(i));
  EXPECT_FALSE(graph_0_->CloseIfNot(i));

  i = graph_1_->GenerateNode(-1, -1, state_1_);
  std::vector<int> tmp_state_1(state_1_.size());
  EXPECT_TRUE(graph_1_->CloseIfNot(i));
  EXPECT_FALSE(graph_1_->CloseIfNot(i));
  state_1_[1] = 1;
  i = graph_1_->GenerateNode(-1, -1, state_1_);
  EXPECT_TRUE(graph_1_->CloseIfNot(i));
  EXPECT_FALSE(graph_1_->CloseIfNot(i));
}

TEST_F(SearchGraphTest, ExtractPathWorks) {
  int node = graph_0_->GenerateNode(-1, -1, state_0_);
  sas_0_->ApplyEffect(4, state_0_);
  node = graph_0_->GenerateNode(4, node, state_0_);
  sas_0_->ApplyEffect(2, state_0_);
  node = graph_0_->GenerateNode(2, node, state_0_);
  sas_0_->ApplyEffect(1, state_0_);
  node = graph_0_->GenerateNode(1, node, state_0_);
  auto result = ExtractPath(graph_0_, node);
  ASSERT_EQ(3, result.size());
  EXPECT_EQ(4, result[0]);
  EXPECT_EQ(2, result[1]);
  EXPECT_EQ(1, result[2]);
}

std::queue<std::string> ExampleSASPlusLines() {
  std::queue<std::string> q;

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push("0");
  q.push("end_metric");
  q.push("3");
  q.push("begin_variable");
  q.push("var0");
  q.push("-1");
  q.push("2");
  q.push("Atom at-robby(rooma)");
  q.push("Atom at-robby(roomb)");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var1");
  q.push("-1");
  q.push("2");
  q.push("Atom carry(ball1, left)");
  q.push("Atom free(left)");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var2");
  q.push("-1");
  q.push("3");
  q.push("Atom at(ball1, rooma)");
  q.push("Atom at(ball1, roomb)");
  q.push("<none of those>");
  q.push("end_variable");
  q.push("1");
  q.push("begin_mutex_group");
  q.push("3");
  q.push("2 0");
  q.push("2 1");
  q.push("1 0");
  q.push("end_mutex_group");
  q.push("begin_state");
  q.push("0");
  q.push("1");
  q.push("0");
  q.push("end_state");
  q.push("begin_goal");
  q.push("1");
  q.push("2 1");
  q.push("end_goal");
  q.push("6");
  q.push("begin_operator");
  q.push("drop ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 -1 0");
  q.push("0 1 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("drop ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 -1 1");
  q.push("0 1 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move rooma roomb");
  q.push("0");
  q.push("1");
  q.push("0 0 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move roomb rooma");
  q.push("0");
  q.push("1");
  q.push("0 0 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 0 2");
  q.push("0 1 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 1 2");
  q.push("0 1 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("0");

  return q;
}

std::queue<std::string> RandomSASPlusLines(std::vector<int> &ranges) {
  std::queue<std::string> q;

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push("0");
  q.push("end_metric");

  int var_max = rand() % 200 + 3;
  q.push(std::to_string(var_max));

  q.push("begin_variable");
  q.push("var0");
  q.push("-1");
  q.push("2");
  q.push("Atom at-robby(rooma)");
  q.push("Atom at-robby(roomb)");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var1");
  q.push("-1");
  q.push("2");
  q.push("Atom carry(ball1, left)");
  q.push("Atom free(left)");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var2");
  q.push("-1");
  q.push("3");
  q.push("Atom at(ball1, rooma)");
  q.push("Atom at(ball1, roomb)");
  q.push("<none of those>");
  q.push("end_variable");

  ranges = std::vector<int>{2, 2, 3};

  for (int i=3; i<var_max; ++i) {
    q.push("begin_variable");
    q.push("var" + std::to_string(i));
    q.push("-1");

    int range = rand() % 20 + 1;
    ranges.push_back(range);
    q.push(std::to_string(range));

    for (int j=0; j<range; ++j)
      q.push("Atom dummy(ball" + std::to_string(j) + ")");

    q.push("end_variable");
  }

  q.push("1");
  q.push("begin_mutex_group");
  q.push("3");
  q.push("2 0");
  q.push("2 1");
  q.push("1 0");
  q.push("end_mutex_group");
  q.push("begin_state");
  q.push("0");
  q.push("1");
  q.push("0");

  for (int i=3; i<var_max; ++i)
    q.push("0");

  q.push("end_state");
  q.push("begin_goal");
  q.push("1");
  q.push("2 1");
  q.push("end_goal");
  q.push("6");
  q.push("begin_operator");
  q.push("drop ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 -1 0");
  q.push("0 1 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("drop ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 -1 1");
  q.push("0 1 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move rooma roomb");
  q.push("0");
  q.push("1");
  q.push("0 0 0 1");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("move roomb rooma");
  q.push("0");
  q.push("1");
  q.push("0 0 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 rooma left");
  q.push("1");
  q.push("0 0");
  q.push("2");
  q.push("0 2 0 2");
  q.push("0 1 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("begin_operator");
  q.push("pick ball1 roomb left");
  q.push("1");
  q.push("0 1");
  q.push("2");
  q.push("0 2 1 2");
  q.push("0 1 1 0");
  q.push("1");
  q.push("end_operator");
  q.push("0");

  return q;
}

} // namespace pplanner
