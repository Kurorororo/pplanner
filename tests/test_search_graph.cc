#include "search_graph.h"

#include <cstdio>

#include <queue>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

class SearchGraphTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    SASPlus sas_0;
    sas_0.InitFromLines(lines);
    graph_0_ = SearchGraph(sas_0);
    state_0_ = std::vector<int>{0, 1, 0};
    state_1_ = std::vector<int>{0, 0, 0};
  }

  SearchGraph graph_0_;
  std::vector<int> state_0_;
  std::vector<int> state_1_;
};

TEST_F(SearchGraphTest, GenerateNodeTest) {
  int node = graph_0_.GenerateNode(state_0_, -1, -1);
  EXPECT_EQ(0, node);
  node = graph_0_.GenerateNode(state_1_, node, 4);
  EXPECT_EQ(1, node);
}

TEST_F(SearchGraphTest, ActionTest) {
  int node = graph_0_.GenerateNode(state_0_, -1, -1);
  EXPECT_EQ(-1, graph_0_.Action(node));
  node = graph_0_.GenerateNode(state_1_, node, 4);
  EXPECT_EQ(4, graph_0_.Action(node));
}

TEST_F(SearchGraphTest, ParentTest) {
  int node = graph_0_.GenerateNode(state_0_, -1, -1);
  EXPECT_EQ(-1, graph_0_.Parent(node));
  node = graph_0_.GenerateNode(state_1_, node, 4);
  EXPECT_EQ(0, graph_0_.Parent(node));
}

TEST_F(SearchGraphTest, CloseWokrs) {
  ASSERT_FALSE(graph_0_.IsClosed(state_0_));
  int node = graph_0_.GenerateNode(state_0_, -1, -1);
  graph_0_.Close(node);
  EXPECT_TRUE(graph_0_.IsClosed(state_0_));
}

TEST_F(SearchGraphTest, StateWorks) {
  int node = graph_0_.GenerateNode(state_0_, -1, -1);
  std::vector<int> tmp_state(state_0_.size());
  graph_0_.State(node, tmp_state);
  EXPECT_EQ(state_0_, tmp_state);
  node = graph_0_.GenerateNode(state_1_, node, 4);
  graph_0_.State(node, tmp_state);
  EXPECT_EQ(state_1_, tmp_state);
}

TEST_F(SearchGraphTest, ExtractPathWorks) {
  int node = graph_0_.GenerateNode(state_0_, -1, -1);
  node = graph_0_.GenerateNode(state_1_, node, 4);
  std::vector<int> tmp_state(state_1_);
  tmp_state[0] = 1;
  tmp_state[3] = 2;
  node = graph_0_.GenerateNode(tmp_state, node, 2);
  tmp_state[1] = 0;
  tmp_state[3] = 1;
  node = graph_0_.GenerateNode(tmp_state, node, 1);
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

} // namespace pplanner
