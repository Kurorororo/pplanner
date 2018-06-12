#include "landmark/landmark_graph.h"

#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "sas_plus.h"
#include "landmark/landmark.h"

namespace pplanner {

std::queue<std::string> ExampleSASPlusLines();

class LandmarkGraphTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    auto sas = std::make_shared<SASPlus>();
    auto lines = ExampleSASPlusLines();
    sas->InitFromLines(lines);

    l_0_ = std::make_shared<Landmark>(0, 2);
    std::vector<std::pair<int, int> > v_1{
      std::make_pair(1, 4), std::make_pair(2, 3)};
    l_1_ = std::make_shared<Landmark>(v_1);
    std::vector<std::pair<int, int> > v_2{
      std::make_pair(0, 1), std::make_pair(2, 5)};
    l_2_ = std::make_shared<Landmark>(v_2);
    l_3_ = std::make_shared<Landmark>(3, 1);
    l_4_ = std::make_shared<Landmark>(1, 2);
    l_5_ = std::make_shared<Landmark>(4, 1);
    l_6_ = std::make_shared<Landmark>(3, 0);

    graph_0_ = std::make_shared<LandmarkGraph>(sas);

    graph_1_ = std::make_shared<LandmarkGraph>(sas);
    graph_1_->Add(*l_0_);
    graph_1_->Add(*l_1_);
    graph_1_->Add(*l_2_);
    graph_1_->Add(*l_3_);

    id_0_ = graph_1_->ToId(*l_0_);
    id_1_ = graph_1_->ToId(*l_1_);
    id_2_ = graph_1_->ToId(*l_2_);
    id_3_ = graph_1_->ToId(*l_3_);

    graph_2_ = std::make_shared<LandmarkGraph>(sas);
    graph_2_->Add(*l_0_);
    graph_2_->Add(*l_1_);
    graph_2_->Add(*l_2_);
    graph_2_->Add(*l_3_);
    graph_2_->Add(*l_4_);
    graph_2_->Add(*l_5_);
    graph_2_->Add(*l_6_);
  }

  std::shared_ptr<LandmarkGraph> graph_0_;
  std::shared_ptr<LandmarkGraph> graph_1_;
  std::shared_ptr<LandmarkGraph> graph_2_;
  std::shared_ptr<Landmark> l_0_;
  std::shared_ptr<Landmark> l_1_;
  std::shared_ptr<Landmark> l_2_;
  std::shared_ptr<Landmark> l_3_;
  std::shared_ptr<Landmark> l_4_;
  std::shared_ptr<Landmark> l_5_;
  std::shared_ptr<Landmark> l_6_;
  int id_0_;
  int id_1_;
  int id_2_;
  int id_3_;
};

TEST_F(LandmarkGraphTest, Landmarks) {
  EXPECT_EQ(0, graph_0_->n_landmarks());

  graph_0_->Add(*l_0_);
  EXPECT_EQ(1, graph_0_->n_landmarks());
  EXPECT_TRUE(graph_0_->IsIn(*l_0_));
  EXPECT_EQ(0, graph_0_->ToId(*l_0_));
  EXPECT_EQ(*l_0_, graph_0_->GetLandmark(0));
  EXPECT_EQ(*l_0_, graph_0_->CopyLandmark(0));

  Landmark l(*l_1_);
  EXPECT_FALSE(graph_0_->IsIn(*l_1_));
  graph_0_->Add(l);
  EXPECT_EQ(2, graph_0_->n_landmarks());
  EXPECT_TRUE(graph_0_->IsIn(*l_1_));
  EXPECT_EQ(1, graph_0_->ToId(*l_1_));
  EXPECT_EQ(*l_1_, graph_0_->GetLandmark(1));
  EXPECT_EQ(*l_1_, graph_0_->CopyLandmark(1));
  EXPECT_EQ(*l_0_, graph_0_->GetLandmarks()[0]);
  EXPECT_EQ(*l_1_, graph_0_->GetLandmarks()[1]);

  graph_0_->Delete(*l_0_);
  EXPECT_FALSE(graph_0_->IsIn(*l_0_));
  EXPECT_TRUE(graph_0_->GetLandmarks()[0].IsEmpty());

  graph_0_->Delete(1);
  EXPECT_FALSE(graph_0_->IsIn(*l_1_));
  EXPECT_TRUE(graph_0_->GetLandmarks()[1].IsEmpty());
}

TEST_F(LandmarkGraphTest, Orderings) {
  /*
  0 -> 1 ->_{gn} 2
        |        A
         ->_r 3__|o
  */
  EXPECT_TRUE(graph_1_->GetTermIdsByInitId(id_0_).empty());
  EXPECT_TRUE(graph_1_->GetTermIdsByInitId(id_1_).empty());
  EXPECT_TRUE(graph_1_->GetTermIdsByInitId(id_2_).empty());
  EXPECT_TRUE(graph_1_->GetTermIdsByInitId(id_3_).empty());

  EXPECT_TRUE(graph_1_->GetInitIdsByTermId(id_0_).empty());
  EXPECT_TRUE(graph_1_->GetInitIdsByTermId(id_1_).empty());
  EXPECT_TRUE(graph_1_->GetInitIdsByTermId(id_2_).empty());
  EXPECT_TRUE(graph_1_->GetInitIdsByTermId(id_3_).empty());

  EXPECT_EQ(0, graph_1_->n_orderings());

  EXPECT_FALSE(graph_1_->IsAdjacent(id_0_, id_1_));
  graph_1_->AddOrdering(id_0_, id_1_, LandmarkGraph::NATURAL);
  EXPECT_EQ(1, graph_1_->n_orderings());
  EXPECT_TRUE(graph_1_->IsAdjacent(id_0_, id_1_));
  EXPECT_FALSE(graph_1_->IsAdjacent(id_1_, id_0_));
  EXPECT_EQ(LandmarkGraph::NATURAL , graph_1_->GetOrderingType(id_0_, id_1_));

  EXPECT_EQ(1, graph_1_->GetTermIdsByInitId(id_0_).size());
  EXPECT_EQ(id_1_, graph_1_->GetTermIdsByInitId(id_0_)[0]);
  EXPECT_EQ(1, graph_1_->GetInitIdsByTermId(id_1_).size());
  EXPECT_EQ(id_0_, graph_1_->GetInitIdsByTermId(id_1_)[0]);

  EXPECT_FALSE(graph_1_->IsAdjacent(id_1_, id_2_));
  graph_1_->AddOrdering(id_1_, id_2_, LandmarkGraph::GREEDY);
  EXPECT_EQ(2, graph_1_->n_orderings());
  EXPECT_TRUE(graph_1_->IsAdjacent(id_1_, id_2_));
  EXPECT_FALSE(graph_1_->IsAdjacent(id_2_, id_1_));
  EXPECT_EQ(LandmarkGraph::GREEDY, graph_1_->GetOrderingType(id_1_, id_2_));

  EXPECT_EQ(1,graph_1_->GetTermIdsByInitId(id_1_).size());
  EXPECT_EQ(id_2_, graph_1_->GetTermIdsByInitId(id_1_)[0]);
  EXPECT_EQ(1, graph_1_->GetInitIdsByTermId(id_2_).size());
  EXPECT_EQ(id_1_, graph_1_->GetInitIdsByTermId(id_2_)[0]);

  EXPECT_FALSE(graph_1_->IsAdjacent(id_1_, id_3_));
  graph_1_->AddOrdering(id_1_, id_3_, LandmarkGraph::REASONABLE);
  EXPECT_EQ(3, graph_1_->n_orderings());
  EXPECT_TRUE(graph_1_->IsAdjacent(id_1_, id_3_));
  EXPECT_EQ(LandmarkGraph::REASONABLE, graph_1_->GetOrderingType(id_1_, id_3_));

  EXPECT_EQ(2, graph_1_->GetTermIdsByInitId(id_1_).size());
  EXPECT_EQ(id_3_, graph_1_->GetTermIdsByInitId(id_1_)[1]);
  EXPECT_EQ(1, graph_1_->GetInitIdsByTermId(id_3_).size());
  EXPECT_EQ(id_1_, graph_1_->GetInitIdsByTermId(id_3_)[0]);

  EXPECT_FALSE(graph_1_->IsAdjacent(id_3_, id_2_));
  graph_1_->AddOrdering(id_3_, id_2_, LandmarkGraph::OBEDIENT);
  EXPECT_EQ(4, graph_1_->n_orderings());
  EXPECT_TRUE(graph_1_->IsAdjacent(id_3_, id_2_));
  EXPECT_EQ(LandmarkGraph::OBEDIENT, graph_1_->GetOrderingType(id_3_, id_2_));

  EXPECT_EQ(1, graph_1_->GetTermIdsByInitId(id_3_).size());
  EXPECT_EQ(id_2_, graph_1_->GetTermIdsByInitId(id_3_)[0]);
  EXPECT_EQ(2, graph_1_->GetInitIdsByTermId(id_2_).size());
  EXPECT_EQ(id_3_, graph_1_->GetInitIdsByTermId(id_2_)[1]);

  graph_1_->DeleteOrdering(id_1_, id_3_);
  EXPECT_EQ(3, graph_1_->n_orderings());
  EXPECT_FALSE(graph_1_->IsAdjacent(id_1_, id_3_));
  EXPECT_EQ(1, graph_1_->GetTermIdsByInitId(id_1_).size());
  EXPECT_EQ(id_2_, graph_1_->GetTermIdsByInitId(id_1_)[0]);
  EXPECT_TRUE(graph_1_->GetInitIdsByTermId(id_3_).empty());

  graph_1_->Delete(id_1_);
  EXPECT_EQ(1, graph_1_->n_orderings());
  EXPECT_TRUE(graph_1_->GetTermIdsByInitId(id_0_).empty());
  EXPECT_EQ(1, graph_1_->GetInitIdsByTermId(id_2_).size());
}

TEST_F(LandmarkGraphTest, GetAncestorsWorks) {
  /*
  0 -> 1 ->_{gn} 2
        |        A
         ->_r 3__|o
  */
  LandmarkGraph::OrderingType natural = LandmarkGraph::NATURAL;
  LandmarkGraph::OrderingType greedy = LandmarkGraph::GREEDY;
  LandmarkGraph::OrderingType reasonable = LandmarkGraph::REASONABLE;
  LandmarkGraph::OrderingType obedient = LandmarkGraph::OBEDIENT;

  graph_1_->AddOrdering(id_0_, id_1_, natural);
  graph_1_->AddOrdering(id_1_, id_2_, greedy);
  graph_1_->AddOrdering(id_1_, id_3_, reasonable);
  graph_1_->AddOrdering(id_3_, id_2_, obedient);

  std::vector<int> ancestors;
  graph_1_->GetAncestors(id_0_, ancestors);
  EXPECT_TRUE(ancestors.empty());
  graph_1_->GetAncestors(id_1_, ancestors);
  EXPECT_EQ(id_1_, ancestors.size());
  EXPECT_EQ(id_0_, ancestors[0]);
  graph_1_->GetAncestors(id_2_, ancestors);
  EXPECT_EQ(id_3_, ancestors.size());
  auto result = std::find(ancestors.begin(), ancestors.end(), id_0_);
  EXPECT_NE(result, ancestors.end());
  result = std::find(ancestors.begin(), ancestors.end(), id_1_);
  EXPECT_NE(result, ancestors.end());
  result = std::find(ancestors.begin(), ancestors.end(), id_3_);
  EXPECT_NE(result, ancestors.end());
  graph_1_->GetAncestors(id_3_, ancestors);
  EXPECT_EQ(id_2_, ancestors.size());

  graph_1_->DeleteOrdering(id_1_, id_3_);
  graph_1_->GetAncestors(id_0_, ancestors);
  EXPECT_TRUE(ancestors.empty());
  graph_1_->GetAncestors(id_1_, ancestors);
  EXPECT_EQ(id_1_, ancestors.size());
  EXPECT_EQ(id_0_, ancestors[0]);
  graph_1_->GetAncestors(id_2_, ancestors);
  EXPECT_EQ(id_3_, ancestors.size());
  result = std::find(ancestors.begin(), ancestors.end(), id_0_);
  EXPECT_NE(result, ancestors.end());
  result = std::find(ancestors.begin(), ancestors.end(), id_1_);
  EXPECT_NE(result, ancestors.end());
  result = std::find(ancestors.begin(), ancestors.end(), id_3_);
  EXPECT_NE(result, ancestors.end());
  graph_1_->GetAncestors(id_3_, ancestors);
  EXPECT_TRUE(ancestors.empty());

  graph_1_->Delete(id_1_);
  graph_1_->GetAncestors(id_0_, ancestors);
  EXPECT_TRUE(ancestors.empty());
  graph_1_->GetAncestors(id_2_, ancestors);
  EXPECT_EQ(id_1_, ancestors.size());
  EXPECT_EQ(id_3_, ancestors[0]);
  graph_1_->GetAncestors(id_3_, ancestors);
  EXPECT_TRUE(ancestors.empty());
}

TEST_F(LandmarkGraphTest, RemoveCyclesWorks) {
  /*        6 < 5
          r | / A
            V   |
  0 -> 1 -> 2 --|
       A        |
       |        | r
       4 <- 3 <-
          o
  */

  int id_0 = graph_2_->ToId(*l_0_);
  int id_1 = graph_2_->ToId(*l_1_);
  int id_2 = graph_2_->ToId(*l_2_);
  int id_3 = graph_2_->ToId(*l_3_);
  int id_4 = graph_2_->ToId(*l_4_);
  int id_5 = graph_2_->ToId(*l_5_);
  int id_6 = graph_2_->ToId(*l_6_);

  graph_2_->AddOrdering(id_0, id_1, LandmarkGraph::GREEDY);
  graph_2_->AddOrdering(id_1, id_2, LandmarkGraph::NATURAL);
  graph_2_->AddOrdering(id_2, id_3, LandmarkGraph::REASONABLE);
  graph_2_->AddOrdering(id_3, id_4, LandmarkGraph::OBEDIENT);
  graph_2_->AddOrdering(id_4, id_1, LandmarkGraph::GREEDY);
  graph_2_->AddOrdering(id_2, id_5, LandmarkGraph::GREEDY);
  graph_2_->AddOrdering(id_5, id_6, LandmarkGraph::NATURAL);
  graph_2_->AddOrdering(id_5, id_2, LandmarkGraph::OBEDIENT);
  graph_2_->AddOrdering(id_6, id_2, LandmarkGraph::REASONABLE);
  EXPECT_TRUE(graph_2_->IsAdjacent(id_0, id_1));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_1, id_2));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_2, id_3));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_3, id_4));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_4, id_1));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_2, id_5));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_5, id_2));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_5, id_6));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_6, id_2));
  EXPECT_TRUE(3 == graph_2_->RemoveCycles(id_0));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_0, id_1));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_1, id_2));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_2, id_3));
  EXPECT_FALSE(graph_2_->IsAdjacent(id_3, id_4));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_4, id_1));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_2, id_5));
  EXPECT_FALSE(graph_2_->IsAdjacent(id_5, id_2));
  EXPECT_TRUE(graph_2_->IsAdjacent(id_5, id_6));
  EXPECT_FALSE(graph_2_->IsAdjacent(id_6, id_2));
}

std::queue<std::string> ExampleSASPlusLines() {
  std::queue<std::string> q;

  q.push("begin_version");
  q.push("3");
  q.push("end_version");
  q.push("begin_metric");
  q.push("1");
  q.push("end_metric");
  q.push("5");
  q.push("begin_variable");
  q.push("var0");
  q.push("-1");
  q.push("3");
  q.push("Atom at-robby(rooma)");
  q.push("Atom at-robby(roomb)");
  q.push("dummy");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var1");
  q.push("-1");
  q.push("5");
  q.push("Atom carry(ball1, left)");
  q.push("Atom free(left)");
  q.push("dummy");
  q.push("dummy");
  q.push("dummy");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var2");
  q.push("-1");
  q.push("6");
  q.push("Atom at(ball1, rooma)");
  q.push("Atom at(ball1, roomb)");
  q.push("<none of those>");
  q.push("dummy");
  q.push("dummy");
  q.push("dummy");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var3");
  q.push("-1");
  q.push("2");
  q.push("dummy");
  q.push("dummy");
  q.push("end_variable");
  q.push("begin_variable");
  q.push("var4");
  q.push("-1");
  q.push("2");
  q.push("dummy");
  q.push("dummy");
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
  q.push("0");
  q.push("0");
  q.push("end_state");
  q.push("begin_goal");
  q.push("1");
  q.push("2 1");
  q.push("end_goal");
  q.push("7");
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
  q.push("begin_operator");
  q.push("dumy-pick ball1 roomb left");
  q.push("2");
  q.push("0 1");
  q.push("2 1");
  q.push("1");
  q.push("0 1 1 0");
  q.push("2");
  q.push("end_operator");
  q.push("0");

  return q;
}

} // namespace pplanner
