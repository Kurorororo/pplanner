#include "open_lists/single_open_list.h"

#include <memory>
#include <queue>
#include <vector>

#include "gtest/gtest.h"

#include "evaluator.h"
#include "sas_plus.h"
#include "search_graph.h"
#include "heuristics/blind.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

class SingleOpenListTest: public ::testing::Test {
 protected:
  virtual void SetUp() {
    list_0_ = std::unique_ptr<SingleOpenList>(new SingleOpenList("fifo"));

    std::vector<std::shared_ptr<Evaluator> > evaluators;
    auto lines = ExampleSASPlusLines();
    auto sas = std::make_shared<SASPlus>();
    sas->InitFromLines(lines);
    auto graph = std::make_shared<SearchGraph>(*sas);
    evaluators.push_back(std::make_shared<Blind>(sas, graph));
    list_1_ = std::unique_ptr<SingleOpenList>(
       new  SingleOpenList("fifo", evaluators));

    state_ = sas->initial();
  }

  std::unique_ptr<SingleOpenList> list_0_;
  std::unique_ptr<SingleOpenList> list_1_;
  std::vector<int> state_;
};

TEST_F(SingleOpenListTest, IsEmptyWorks) {
  EXPECT_TRUE(list_0_->IsEmpty());
  EXPECT_TRUE(list_1_->IsEmpty());
}

TEST_F(SingleOpenListTest, PushWorks) {
  int node = 0;
  std::vector<int> values{0, 1, 2};
  list_0_->Push(values, node, false);
  EXPECT_FALSE(list_0_->IsEmpty());
}

TEST_F(SingleOpenListTest, EvaluateAndPushWorks) {
  int node = 0;
  list_1_->Push(state_, node, false);
  EXPECT_FALSE(list_1_->IsEmpty());
}

TEST_F(SingleOpenListTest, PopWorks) {
  int node = 0;
  std::vector<int> values{0, 1, 2};
  list_0_->Push(values, node, false);
  node = 1;
  values[0] = 1;
  list_0_->Push(values, node, false);
  values[1] = 0;
  node = 2;
  list_0_->Push(values, node, false);
  values[0] = 0;
  node = 3;
  list_0_->Push(values, node, false);

  node = list_0_->Pop();
  EXPECT_EQ(3, node);
  node = list_0_->Pop();
  EXPECT_EQ(0, node);
  node = list_0_->Pop();
  EXPECT_EQ(2, node);
  node = list_0_->Pop();
  EXPECT_EQ(1, node);
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
