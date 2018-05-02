#include "heuristics/blind.h"

#include <memory>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"

#include "sas_plus.h"
#include "search_graph.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

class BlindTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto lines = ExampleSASPlusLines();
    auto sas = std::make_shared<SASPlus>();
    sas->InitFromLines(lines);
    auto graph = std::make_shared<SearchGraph>(*sas);
    blind_ = Blind(sas, graph);

    state = sas->initial();
  }

  Blind blind_;
  std::vector<int> state;
  std::vector<int> applicable;
  std::unordered_set<int> preferred;
};

TEST_F(BlindTest, EvaluateWorks) {
  int h = blind_.Evaluate(state, 0);
  EXPECT_EQ(1, h);
  blind_.Evaluate(state, 0, applicable, preferred);
  EXPECT_TRUE(preferred.empty());
  state[1] = 0;
  h = blind_.Evaluate(state, 0);
  EXPECT_EQ(1, h);
  state[0] = 1;
  state[2] = 2;
  h = blind_.Evaluate(state, 0);
  EXPECT_EQ(1, h);
  state[1] = 1;
  state[2] = 1;
  h = blind_.Evaluate(state, 0);
  EXPECT_EQ(0, h);
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
