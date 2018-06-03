#include "landmark/landmark_graph.h"

#include <iostream>
#include <vector>

#include "domain/var_value.h"
#include "landmark/landmark.h"

using namespace rwls;

void TestConstructor() {
  LandmarkGraph graph;

  std::cout << "passed Constructor" << std::endl;
}

void TestLandmarks() {
  LandmarkGraph graph;

  assert(0 == graph.GetLandmarksSize());

  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  graph.Add(landmark);
  assert(1 == graph.GetLandmarksSize());
  assert(graph.IsIn(landmark));
  assert(0 == graph.ToId(landmark));
  assert(landmark == graph.GetLandmark(0));
  assert(landmark == graph.CopyLandmark(0));

  EncodeVarValue(1, 4, &v);
  Landmark landmark2(v);
  EncodeVarValue(2, 3, &v);
  landmark2.AddVarValue(v);
  Landmark landmark3(landmark2);
  assert(!graph.IsIn(landmark2));
  graph.Add(std::move(landmark3));
  assert(2 == graph.GetLandmarksSize());
  assert(graph.IsIn(landmark2));
  assert(1 == graph.ToId(landmark2));
  assert(landmark2 == graph.GetLandmark(1));
  assert(landmark2 == graph.CopyLandmark(1));
  assert(landmark == graph.GetLandmarks()[0]);
  assert(landmark2 == graph.GetLandmarks()[1]);

  graph.Delete(landmark);
  assert(!graph.IsIn(landmark));
  assert(graph.GetLandmarks()[0].IsEmpty());

  graph.Delete(1);
  assert(!graph.IsIn(landmark2));
  assert(graph.GetLandmarks()[1].IsEmpty());

  std::cout << "passed Landmarks" << std::endl;
}

void TestOrderings() {
  /*
  0 -> 1 ->_{gn} 2
        |        A
         ->_r 3__|o
  */

  LandmarkGraph graph;

  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  graph.Add(landmark);
  EncodeVarValue(1, 4, &v);
  Landmark landmark1(v);
  EncodeVarValue(2, 3, &v);
  landmark1.AddVarValue(v);
  graph.Add(landmark1);
  EncodeVarValue(0, 1, &v);
  Landmark landmark2(v);
  EncodeVarValue(2, 5, &v);
  landmark2.AddVarValue(v);
  graph.Add(landmark2);
  EncodeVarValue(3, 1, &v);
  Landmark landmark3(v);
  landmark3.AddVarValue(v);
  graph.Add(landmark3);

  size_t id_0 = graph.ToId(landmark);
  size_t id_1 = graph.ToId(landmark1);
  size_t id_2 = graph.ToId(landmark2);
  size_t id_3 = graph.ToId(landmark3);

  assert(graph.GetTermIdsByInitId(id_0).empty());
  assert(graph.GetTermIdsByInitId(id_1).empty());
  assert(graph.GetTermIdsByInitId(id_2).empty());
  assert(graph.GetTermIdsByInitId(id_3).empty());

  assert(graph.GetInitIdsByTermId(id_0).empty());
  assert(graph.GetInitIdsByTermId(id_1).empty());
  assert(graph.GetInitIdsByTermId(id_2).empty());
  assert(graph.GetInitIdsByTermId(id_3).empty());

  assert(0 == graph.GetOrderingsSize());

  assert(!graph.IsAdjacent(id_0, id_1));
  graph.AddOrdering(id_0, id_1, LandmarkGraph::NATURAL);
  assert(1 == graph.GetOrderingsSize());
  assert(graph.IsAdjacent(id_0, id_1));
  assert(!graph.IsAdjacent(id_1, id_0));
  assert(LandmarkGraph::NATURAL == graph.GetOrderingType(id_0, id_1));

  assert(1 == graph.GetTermIdsByInitId(id_0).size());
  assert(id_1 == graph.GetTermIdsByInitId(id_0)[0]);
  assert(1 == graph.GetInitIdsByTermId(id_1).size());
  assert(id_0 == graph.GetInitIdsByTermId(id_1)[0]);

  assert(!graph.IsAdjacent(id_1, id_2));
  graph.AddOrdering(id_1, id_2, LandmarkGraph::GREEDY);
  assert(2 == graph.GetOrderingsSize());
  assert(graph.IsAdjacent(id_1, id_2));
  assert(!graph.IsAdjacent(id_2, id_1));
  assert(LandmarkGraph::GREEDY == graph.GetOrderingType(id_1, id_2));

  assert(1 == graph.GetTermIdsByInitId(id_1).size());
  assert(id_2 == graph.GetTermIdsByInitId(id_1)[0]);
  assert(1 == graph.GetInitIdsByTermId(id_2).size());
  assert(id_1 == graph.GetInitIdsByTermId(id_2)[0]);

  assert(!graph.IsAdjacent(id_1, id_3));
  graph.AddOrdering(id_1, id_3, LandmarkGraph::REASONABLE);
  assert(3 == graph.GetOrderingsSize());
  assert(graph.IsAdjacent(id_1, id_3));
  assert(LandmarkGraph::REASONABLE == graph.GetOrderingType(id_1, id_3));

  assert(2 == graph.GetTermIdsByInitId(id_1).size());
  assert(id_3 == graph.GetTermIdsByInitId(id_1)[1]);
  assert(1 == graph.GetInitIdsByTermId(id_3).size());
  assert(id_1 == graph.GetInitIdsByTermId(id_3)[0]);

  assert(!graph.IsAdjacent(id_3, id_2));
  graph.AddOrdering(id_3, id_2, LandmarkGraph::OBEDIENT);
  assert(4 == graph.GetOrderingsSize());
  assert(graph.IsAdjacent(id_3, id_2));
  assert(LandmarkGraph::OBEDIENT == graph.GetOrderingType(id_3, id_2));

  assert(1 == graph.GetTermIdsByInitId(id_3).size());
  assert(id_2 == graph.GetTermIdsByInitId(id_3)[0]);
  assert(2 == graph.GetInitIdsByTermId(id_2).size());
  assert(id_3 == graph.GetInitIdsByTermId(id_2)[1]);

  graph.DeleteOrdering(id_1, id_3);
  assert(3 == graph.GetOrderingsSize());
  assert(!graph.IsAdjacent(id_1, id_3));
  assert(1 == graph.GetTermIdsByInitId(id_1).size());
  assert(id_2 == graph.GetTermIdsByInitId(id_1)[0]);
  assert(graph.GetInitIdsByTermId(id_3).empty());

  graph.Delete(id_1);
  assert(1 == graph.GetOrderingsSize());
  assert(graph.GetTermIdsByInitId(id_0).empty());
  assert(1 == graph.GetInitIdsByTermId(id_2).size());

  std::cout << "passed Orderings" << std::endl;
}

void TestGetAncestors() {
  /*
  0 -> 1 ->_{gn} 2
        |        A
         ->_r 3__|o
  */

  LandmarkGraph graph;

  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  graph.Add(landmark);
  EncodeVarValue(1, 4, &v);
  Landmark landmark1(v);
  EncodeVarValue(2, 3, &v);
  landmark1.AddVarValue(v);
  graph.Add(landmark1);
  EncodeVarValue(0, 1, &v);
  Landmark landmark2(v);
  EncodeVarValue(2, 5, &v);
  landmark2.AddVarValue(v);
  graph.Add(landmark2);
  EncodeVarValue(3, 1, &v);
  Landmark landmark3(v);
  landmark3.AddVarValue(v);
  graph.Add(landmark3);

  size_t id_0 = graph.ToId(landmark);
  size_t id_1 = graph.ToId(landmark1);
  size_t id_2 = graph.ToId(landmark2);
  size_t id_3 = graph.ToId(landmark3);

  int natural = LandmarkGraph::NATURAL;
  int greedy = LandmarkGraph::GREEDY;
  int reasonable = LandmarkGraph::REASONABLE;
  int obedient = LandmarkGraph::OBEDIENT;

  graph.AddOrdering(id_0, id_1, natural);
  graph.AddOrdering(id_1, id_2, greedy);
  graph.AddOrdering(id_1, id_3, reasonable);
  graph.AddOrdering(id_3, id_2, obedient);

  std::vector<size_t> ancestors;
  graph.GetAncestors(id_0, ancestors);
  assert(ancestors.empty());
  graph.GetAncestors(id_1, ancestors);
  assert(id_1 == ancestors.size());
  assert(id_0 == ancestors[0]);
  graph.GetAncestors(id_2, ancestors);
  assert(id_3 == ancestors.size());
  auto result = std::find(ancestors.begin(), ancestors.end(), id_0);
  assert(result != ancestors.end());
  result = std::find(ancestors.begin(), ancestors.end(), id_1);
  assert(result != ancestors.end());
  result = std::find(ancestors.begin(), ancestors.end(), id_3);
  assert(result != ancestors.end());
  graph.GetAncestors(id_3, ancestors);
  assert(id_2 == ancestors.size());
  result = std::find(ancestors.begin(), ancestors.end(), id_1);
  result = std::find(ancestors.begin(), ancestors.end(), id_0);

  graph.DeleteOrdering(id_1, id_3);
  graph.GetAncestors(id_0, ancestors);
  assert(ancestors.empty());
  graph.GetAncestors(id_1, ancestors);
  assert(id_1 == ancestors.size());
  assert(id_0 == ancestors[0]);
  graph.GetAncestors(id_2, ancestors);
  assert(id_3 == ancestors.size());
  result = std::find(ancestors.begin(), ancestors.end(), id_0);
  assert(result != ancestors.end());
  result = std::find(ancestors.begin(), ancestors.end(), id_1);
  assert(result != ancestors.end());
  result = std::find(ancestors.begin(), ancestors.end(), id_3);
  assert(result != ancestors.end());
  graph.GetAncestors(id_3, ancestors);
  assert(ancestors.empty());

  graph.Delete(id_1);
  graph.GetAncestors(id_0, ancestors);
  assert(ancestors.empty());
  graph.GetAncestors(id_2, ancestors);
  assert(id_1 == ancestors.size());
  assert(id_3 == ancestors[0]);
  graph.GetAncestors(id_3, ancestors);
  assert(ancestors.empty());

  std::cout << "passed GetAncestors" << std::endl;
}

void TestRemoveCycles() {
  /*        6 < 5
          r | / A
            V   |
  0 -> 1 -> 2 --|
       A        |
       |        | r
       4 <- 3 <-
          o
  */

  LandmarkGraph graph;

  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  graph.Add(landmark);
  EncodeVarValue(1, 4, &v);
  Landmark landmark1(v);
  EncodeVarValue(2, 3, &v);
  landmark1.AddVarValue(v);
  graph.Add(landmark1);
  EncodeVarValue(0, 1, &v);
  Landmark landmark2(v);
  EncodeVarValue(2, 5, &v);
  landmark2.AddVarValue(v);
  graph.Add(landmark2);
  EncodeVarValue(3, 1, &v);
  Landmark landmark3(v);
  landmark3.AddVarValue(v);
  graph.Add(landmark3);
  EncodeVarValue(1, 2, &v);
  Landmark landmark4(v);
  landmark4.AddVarValue(v);
  graph.Add(landmark4);
  EncodeVarValue(4, 1, &v);
  Landmark landmark5(v);
  landmark5.AddVarValue(v);
  graph.Add(landmark5);
  EncodeVarValue(3, 0, &v);
  Landmark landmark6(v);
  landmark6.AddVarValue(v);
  graph.Add(landmark6);

  size_t id_0 = graph.ToId(landmark);
  size_t id_1 = graph.ToId(landmark1);
  size_t id_2 = graph.ToId(landmark2);
  size_t id_3 = graph.ToId(landmark3);
  size_t id_4 = graph.ToId(landmark4);
  size_t id_5 = graph.ToId(landmark5);
  size_t id_6 = graph.ToId(landmark6);

  graph.AddOrdering(id_0, id_1, LandmarkGraph::GREEDY);
  graph.AddOrdering(id_1, id_2, LandmarkGraph::NATURAL);
  graph.AddOrdering(id_2, id_3, LandmarkGraph::REASONABLE);
  graph.AddOrdering(id_3, id_4, LandmarkGraph::OBEDIENT);
  graph.AddOrdering(id_4, id_1, LandmarkGraph::GREEDY);
  graph.AddOrdering(id_2, id_5, LandmarkGraph::GREEDY);
  graph.AddOrdering(id_5, id_6, LandmarkGraph::NATURAL);
  graph.AddOrdering(id_5, id_2, LandmarkGraph::OBEDIENT);
  graph.AddOrdering(id_6, id_2, LandmarkGraph::REASONABLE);

  assert(graph.IsAdjacent(id_0, id_1));
  assert(graph.IsAdjacent(id_1, id_2));
  assert(graph.IsAdjacent(id_2, id_3));
  assert(graph.IsAdjacent(id_3, id_4));
  assert(graph.IsAdjacent(id_4, id_1));
  assert(graph.IsAdjacent(id_2, id_5));
  assert(graph.IsAdjacent(id_5, id_2));
  assert(graph.IsAdjacent(id_5, id_6));
  assert(graph.IsAdjacent(id_6, id_2));
  assert(3 == graph.RemoveCycles(id_0));
  assert(graph.IsAdjacent(id_0, id_1));
  assert(graph.IsAdjacent(id_1, id_2));
  assert(graph.IsAdjacent(id_2, id_3));
  assert(!graph.IsAdjacent(id_3, id_4));
  assert(graph.IsAdjacent(id_4, id_1));
  assert(graph.IsAdjacent(id_2, id_5));
  assert(!graph.IsAdjacent(id_5, id_2));
  assert(graph.IsAdjacent(id_5, id_6));
  assert(!graph.IsAdjacent(id_6, id_2));

  std::cout << "passed RemoveCycle" << std::endl;
}

int main() {
  TestLandmarks();
  TestOrderings();
  TestGetAncestors();
  TestRemoveCycles();
}
