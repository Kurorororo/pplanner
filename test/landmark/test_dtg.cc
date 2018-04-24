#include "landmark/dtg.h"

#include <iostream>
#include <unordered_set>
#include <vector>

#include "heuristic/graphplan.h"

using namespace rwls;

void TestConstructor() {
  DTG dtg;
  std::vector< std::unordered_set<int> > adjacent_lists(4);
  DTG dtg2(adjacent_lists);
  DTG dtg3(dtg2);

  std::cout << "passed TestConstructor" << std::endl;
}

void TestCopy() {
  std::vector< std::unordered_set<int> > adjacent_lists(4);
  DTG dtg(adjacent_lists);
  DTG dtg2;
  dtg2 = dtg;

  std::cout << "passed TestCopy" << std::endl;
}

void TestIsConnected() {
  std::vector< std::unordered_set<int> > adjacent_lists(4);
  /**
   0-->1-->3
    |    |
    -->2--
   */
  adjacent_lists[0].insert(1);
  adjacent_lists[0].insert(2);
  adjacent_lists[1].insert(3);
  adjacent_lists[2].insert(3);
  DTG dtg(adjacent_lists);

  assert(dtg.IsConnected(0, 1));
  assert(dtg.IsConnected(0, 1, 2));
  assert(dtg.IsConnected(0, 1, 3));
  assert(dtg.IsConnected(0, 2));
  assert(dtg.IsConnected(0, 2, 1));
  assert(dtg.IsConnected(0, 2, 3));
  assert(dtg.IsConnected(0, 3));
  assert(dtg.IsConnected(0, 3, 1));
  assert(dtg.IsConnected(0, 3, 2));

  assert(!dtg.IsConnected(1, 0));
  assert(!dtg.IsConnected(1, 0, 2));
  assert(!dtg.IsConnected(1, 0, 3));
  assert(!dtg.IsConnected(1, 2));
  assert(!dtg.IsConnected(1, 2, 0));
  assert(!dtg.IsConnected(1, 2, 3));
  assert(dtg.IsConnected(1, 3));
  assert(dtg.IsConnected(1, 3, 0));
  assert(dtg.IsConnected(1, 3, 2));

  assert(!dtg.IsConnected(2, 0));
  assert(!dtg.IsConnected(2, 0, 1));
  assert(!dtg.IsConnected(2, 0, 3));
  assert(!dtg.IsConnected(2, 1));
  assert(!dtg.IsConnected(2, 1, 0));
  assert(!dtg.IsConnected(2, 1, 3));
  assert(dtg.IsConnected(2, 3));
  assert(dtg.IsConnected(2, 3, 0));
  assert(dtg.IsConnected(2, 3, 1));

  assert(!dtg.IsConnected(3, 0));
  assert(!dtg.IsConnected(3, 0, 1));
  assert(!dtg.IsConnected(3, 0, 2));
  assert(!dtg.IsConnected(3, 1));
  assert(!dtg.IsConnected(3, 1, 0));
  assert(!dtg.IsConnected(3, 1, 2));
  assert(!dtg.IsConnected(3, 2));
  assert(!dtg.IsConnected(3, 2, 0));
  assert(!dtg.IsConnected(3, 2, 1));

  std::cout << "passed IsConnected" << std::endl;
}

void TestRemoveNode() {
  std::vector< std::unordered_set<int> > adjacent_lists(4);

  /**
   0-->1-->3
    |    |
    -->2--
   */
  adjacent_lists[0].insert(1);
  adjacent_lists[0].insert(2);
  adjacent_lists[1].insert(3);
  adjacent_lists[2].insert(3);

  DTG dtg(adjacent_lists);
  dtg.RemoveNode(1);
  assert(!dtg.IsConnected(0, 1));
  assert(!dtg.IsConnected(0, 1, 2));
  assert(!dtg.IsConnected(0, 1, 3));
  assert(dtg.IsConnected(0, 2));
  assert(dtg.IsConnected(0, 2, 1));
  assert(dtg.IsConnected(0, 2, 3));
  assert(dtg.IsConnected(0, 3));
  assert(dtg.IsConnected(0, 3, 1));
  assert(!dtg.IsConnected(0, 3, 2));

  assert(!dtg.IsConnected(1, 0));
  assert(!dtg.IsConnected(1, 0, 2));
  assert(!dtg.IsConnected(1, 0, 3));
  assert(!dtg.IsConnected(1, 2));
  assert(!dtg.IsConnected(1, 2, 0));
  assert(!dtg.IsConnected(1, 2, 3));
  assert(!dtg.IsConnected(1, 3));
  assert(!dtg.IsConnected(1, 3, 0));
  assert(!dtg.IsConnected(1, 3, 2));

  assert(!dtg.IsConnected(2, 0));
  assert(!dtg.IsConnected(2, 0, 1));
  assert(!dtg.IsConnected(2, 0, 3));
  assert(!dtg.IsConnected(2, 1));
  assert(!dtg.IsConnected(2, 1, 0));
  assert(!dtg.IsConnected(2, 1, 3));
  assert(dtg.IsConnected(2, 3));
  assert(dtg.IsConnected(2, 3, 0));
  assert(dtg.IsConnected(2, 3, 1));

  assert(!dtg.IsConnected(3, 0));
  assert(!dtg.IsConnected(3, 0, 1));
  assert(!dtg.IsConnected(3, 0, 2));
  assert(!dtg.IsConnected(3, 1));
  assert(!dtg.IsConnected(3, 1, 0));
  assert(!dtg.IsConnected(3, 1, 2));
  assert(!dtg.IsConnected(3, 2));
  assert(!dtg.IsConnected(3, 2, 0));
  assert(!dtg.IsConnected(3, 2, 1));

  dtg.RemoveNode(2);
  assert(!dtg.IsConnected(0, 1));
  assert(!dtg.IsConnected(0, 1, 2));
  assert(!dtg.IsConnected(0, 1, 3));
  assert(!dtg.IsConnected(0, 2));
  assert(!dtg.IsConnected(0, 2, 1));
  assert(!dtg.IsConnected(0, 2, 3));
  assert(!dtg.IsConnected(0, 3));
  assert(!dtg.IsConnected(0, 3, 1));
  assert(!dtg.IsConnected(0, 3, 2));

  assert(!dtg.IsConnected(1, 0));
  assert(!dtg.IsConnected(1, 0, 2));
  assert(!dtg.IsConnected(1, 0, 3));
  assert(!dtg.IsConnected(1, 2));
  assert(!dtg.IsConnected(1, 2, 0));
  assert(!dtg.IsConnected(1, 2, 3));
  assert(!dtg.IsConnected(1, 3));
  assert(!dtg.IsConnected(1, 3, 0));
  assert(!dtg.IsConnected(1, 3, 2));

  assert(!dtg.IsConnected(2, 0));
  assert(!dtg.IsConnected(2, 0, 1));
  assert(!dtg.IsConnected(2, 0, 3));
  assert(!dtg.IsConnected(2, 1));
  assert(!dtg.IsConnected(2, 1, 0));
  assert(!dtg.IsConnected(2, 1, 3));
  assert(!dtg.IsConnected(2, 3));
  assert(!dtg.IsConnected(2, 3, 0));
  assert(!dtg.IsConnected(2, 3, 1));

  assert(!dtg.IsConnected(3, 0));
  assert(!dtg.IsConnected(3, 0, 1));
  assert(!dtg.IsConnected(3, 0, 2));
  assert(!dtg.IsConnected(3, 1));
  assert(!dtg.IsConnected(3, 1, 0));
  assert(!dtg.IsConnected(3, 1, 2));
  assert(!dtg.IsConnected(3, 2));
  assert(!dtg.IsConnected(3, 2, 0));
  assert(!dtg.IsConnected(3, 2, 1));

  std::cout << "passed RemoveNode" << std::endl;
}


void TestSoftRemoveNode() {
  std::vector< std::unordered_set<int> > adjacent_lists(4);
  /**
   0-->1-->3
    |    |
    -->2--
   */
  adjacent_lists[0].insert(1);
  adjacent_lists[0].insert(2);
  adjacent_lists[1].insert(3);
  adjacent_lists[2].insert(3);
  DTG dtg(adjacent_lists);

  dtg.SoftRemoveNode(1);
  assert(!dtg.IsConnected(0, 1));
  assert(!dtg.IsConnected(0, 1, 2));
  assert(!dtg.IsConnected(0, 1, 3));
  assert(dtg.IsConnected(0, 2));
  assert(dtg.IsConnected(0, 2, 1));
  assert(dtg.IsConnected(0, 2, 3));
  assert(dtg.IsConnected(0, 3));
  assert(dtg.IsConnected(0, 3, 1));
  assert(!dtg.IsConnected(0, 3, 2));

  assert(!dtg.IsConnected(1, 0));
  assert(!dtg.IsConnected(1, 0, 2));
  assert(!dtg.IsConnected(1, 0, 3));
  assert(!dtg.IsConnected(1, 2));
  assert(!dtg.IsConnected(1, 2, 0));
  assert(!dtg.IsConnected(1, 2, 3));
  assert(!dtg.IsConnected(1, 3));
  assert(!dtg.IsConnected(1, 3, 0));
  assert(!dtg.IsConnected(1, 3, 2));

  assert(!dtg.IsConnected(2, 0));
  assert(!dtg.IsConnected(2, 0, 1));
  assert(!dtg.IsConnected(2, 0, 3));
  assert(!dtg.IsConnected(2, 1));
  assert(!dtg.IsConnected(2, 1, 0));
  assert(!dtg.IsConnected(2, 1, 3));
  assert(dtg.IsConnected(2, 3));
  assert(dtg.IsConnected(2, 3, 0));
  assert(dtg.IsConnected(2, 3, 1));

  assert(!dtg.IsConnected(3, 0));
  assert(!dtg.IsConnected(3, 0, 1));
  assert(!dtg.IsConnected(3, 0, 2));
  assert(!dtg.IsConnected(3, 1));
  assert(!dtg.IsConnected(3, 1, 0));
  assert(!dtg.IsConnected(3, 1, 2));
  assert(!dtg.IsConnected(3, 2));
  assert(!dtg.IsConnected(3, 2, 0));
  assert(!dtg.IsConnected(3, 2, 1));

  dtg.SoftRemoveNode(2);
  assert(!dtg.IsConnected(0, 1));
  assert(!dtg.IsConnected(0, 1, 2));
  assert(!dtg.IsConnected(0, 1, 3));
  assert(!dtg.IsConnected(0, 2));
  assert(!dtg.IsConnected(0, 2, 1));
  assert(!dtg.IsConnected(0, 2, 3));
  assert(!dtg.IsConnected(0, 3));
  assert(!dtg.IsConnected(0, 3, 1));
  assert(!dtg.IsConnected(0, 3, 2));

  assert(!dtg.IsConnected(1, 0));
  assert(!dtg.IsConnected(1, 0, 2));
  assert(!dtg.IsConnected(1, 0, 3));
  assert(!dtg.IsConnected(1, 2));
  assert(!dtg.IsConnected(1, 2, 0));
  assert(!dtg.IsConnected(1, 2, 3));
  assert(!dtg.IsConnected(1, 3));
  assert(!dtg.IsConnected(1, 3, 0));
  assert(!dtg.IsConnected(1, 3, 2));

  assert(!dtg.IsConnected(2, 0));
  assert(!dtg.IsConnected(2, 0, 1));
  assert(!dtg.IsConnected(2, 0, 3));
  assert(!dtg.IsConnected(2, 1));
  assert(!dtg.IsConnected(2, 1, 0));
  assert(!dtg.IsConnected(2, 1, 3));
  assert(!dtg.IsConnected(2, 3));
  assert(!dtg.IsConnected(2, 3, 0));
  assert(!dtg.IsConnected(2, 3, 1));

  assert(!dtg.IsConnected(3, 0));
  assert(!dtg.IsConnected(3, 0, 1));
  assert(!dtg.IsConnected(3, 0, 2));
  assert(!dtg.IsConnected(3, 1));
  assert(!dtg.IsConnected(3, 1, 0));
  assert(!dtg.IsConnected(3, 1, 2));
  assert(!dtg.IsConnected(3, 2));
  assert(!dtg.IsConnected(3, 2, 0));
  assert(!dtg.IsConnected(3, 2, 1));

  std::cout << "passed SoftRemoveNode" << std::endl;
}

void TestRemoveNodesByRRPG() {
  std::vector< std::unordered_set<int> > adjacent_lists(4);
  /**
   0-->1-->3
    |    |
    -->2--
   */
  adjacent_lists[0].insert(1);
  adjacent_lists[0].insert(2);
  adjacent_lists[1].insert(3);
  adjacent_lists[2].insert(3);
  DTG dtg(adjacent_lists);

  Domain domain;
  domain.fact_offset.push_back(0);
  domain.dom.push_back(4);
  domain.variables_size = 1;
  domain.fact_size = 4;

  PlanningGraph rrpg;
  rrpg.fact_layer_membership.resize(4, 0);
  rrpg.fact_layer_membership[1] = -1;

  dtg.RemoveNodesByRRPG(domain, rrpg, 0, 3);
  assert(!dtg.IsConnected(0, 1));
  assert(!dtg.IsConnected(0, 1, 2));
  assert(!dtg.IsConnected(0, 1, 3));
  assert(dtg.IsConnected(0, 2));
  assert(dtg.IsConnected(0, 2, 1));
  assert(dtg.IsConnected(0, 2, 3));
  assert(dtg.IsConnected(0, 3));
  assert(dtg.IsConnected(0, 3, 1));
  assert(!dtg.IsConnected(0, 3, 2));

  assert(!dtg.IsConnected(1, 0));
  assert(!dtg.IsConnected(1, 0, 2));
  assert(!dtg.IsConnected(1, 0, 3));
  assert(!dtg.IsConnected(1, 2));
  assert(!dtg.IsConnected(1, 2, 0));
  assert(!dtg.IsConnected(1, 2, 3));
  assert(!dtg.IsConnected(1, 3));
  assert(!dtg.IsConnected(1, 3, 0));
  assert(!dtg.IsConnected(1, 3, 2));

  assert(!dtg.IsConnected(2, 0));
  assert(!dtg.IsConnected(2, 0, 1));
  assert(!dtg.IsConnected(2, 0, 3));
  assert(!dtg.IsConnected(2, 1));
  assert(!dtg.IsConnected(2, 1, 0));
  assert(!dtg.IsConnected(2, 1, 3));
  assert(dtg.IsConnected(2, 3));
  assert(dtg.IsConnected(2, 3, 0));
  assert(dtg.IsConnected(2, 3, 1));

  assert(!dtg.IsConnected(3, 0));
  assert(!dtg.IsConnected(3, 0, 1));
  assert(!dtg.IsConnected(3, 0, 2));
  assert(!dtg.IsConnected(3, 1));
  assert(!dtg.IsConnected(3, 1, 0));
  assert(!dtg.IsConnected(3, 1, 2));
  assert(!dtg.IsConnected(3, 2));
  assert(!dtg.IsConnected(3, 2, 0));
  assert(!dtg.IsConnected(3, 2, 1));

  dtg.RecoverSoftDelete();
  assert(dtg.IsConnected(0, 1));
  assert(dtg.IsConnected(0, 1, 2));
  assert(dtg.IsConnected(0, 1, 3));
  assert(dtg.IsConnected(0, 2));
  assert(dtg.IsConnected(0, 2, 1));
  assert(dtg.IsConnected(0, 2, 3));
  assert(dtg.IsConnected(0, 3));
  assert(dtg.IsConnected(0, 3, 1));
  assert(dtg.IsConnected(0, 3, 2));

  assert(!dtg.IsConnected(1, 0));
  assert(!dtg.IsConnected(1, 0, 2));
  assert(!dtg.IsConnected(1, 0, 3));
  assert(!dtg.IsConnected(1, 2));
  assert(!dtg.IsConnected(1, 2, 0));
  assert(!dtg.IsConnected(1, 2, 3));
  assert(dtg.IsConnected(1, 3));
  assert(dtg.IsConnected(1, 3, 0));
  assert(dtg.IsConnected(1, 3, 2));

  assert(!dtg.IsConnected(2, 0));
  assert(!dtg.IsConnected(2, 0, 1));
  assert(!dtg.IsConnected(2, 0, 3));
  assert(!dtg.IsConnected(2, 1));
  assert(!dtg.IsConnected(2, 1, 0));
  assert(!dtg.IsConnected(2, 1, 3));
  assert(dtg.IsConnected(2, 3));
  assert(dtg.IsConnected(2, 3, 0));
  assert(dtg.IsConnected(2, 3, 1));

  assert(!dtg.IsConnected(3, 0));
  assert(!dtg.IsConnected(3, 0, 1));
  assert(!dtg.IsConnected(3, 0, 2));
  assert(!dtg.IsConnected(3, 1));
  assert(!dtg.IsConnected(3, 1, 0));
  assert(!dtg.IsConnected(3, 1, 2));
  assert(!dtg.IsConnected(3, 2));
  assert(!dtg.IsConnected(3, 2, 0));
  assert(!dtg.IsConnected(3, 2, 1));

  rrpg.fact_layer_membership[2] = -1;
  dtg.RemoveNodesByRRPG(domain, rrpg, 0, 3);
  assert(!dtg.IsConnected(0, 1));
  assert(!dtg.IsConnected(0, 1, 2));
  assert(!dtg.IsConnected(0, 1, 3));
  assert(!dtg.IsConnected(0, 2));
  assert(!dtg.IsConnected(0, 2, 1));
  assert(!dtg.IsConnected(0, 2, 3));
  assert(!dtg.IsConnected(0, 3));
  assert(!dtg.IsConnected(0, 3, 1));
  assert(!dtg.IsConnected(0, 3, 2));

  assert(!dtg.IsConnected(1, 0));
  assert(!dtg.IsConnected(1, 0, 2));
  assert(!dtg.IsConnected(1, 0, 3));
  assert(!dtg.IsConnected(1, 2));
  assert(!dtg.IsConnected(1, 2, 0));
  assert(!dtg.IsConnected(1, 2, 3));
  assert(!dtg.IsConnected(1, 3));
  assert(!dtg.IsConnected(1, 3, 0));
  assert(!dtg.IsConnected(1, 3, 2));

  assert(!dtg.IsConnected(2, 0));
  assert(!dtg.IsConnected(2, 0, 1));
  assert(!dtg.IsConnected(2, 0, 3));
  assert(!dtg.IsConnected(2, 1));
  assert(!dtg.IsConnected(2, 1, 0));
  assert(!dtg.IsConnected(2, 1, 3));
  assert(!dtg.IsConnected(2, 3));
  assert(!dtg.IsConnected(2, 3, 0));
  assert(!dtg.IsConnected(2, 3, 1));

  assert(!dtg.IsConnected(3, 0));
  assert(!dtg.IsConnected(3, 0, 1));
  assert(!dtg.IsConnected(3, 0, 2));
  assert(!dtg.IsConnected(3, 1));
  assert(!dtg.IsConnected(3, 1, 0));
  assert(!dtg.IsConnected(3, 1, 2));
  assert(!dtg.IsConnected(3, 2));
  assert(!dtg.IsConnected(3, 2, 0));
  assert(!dtg.IsConnected(3, 2, 1));

  std::cout << "passed RemoveNodesByRRPG" << std::endl;
}

int main() {
  TestConstructor();
  TestCopy();
  TestIsConnected();
  TestRemoveNode();
  TestSoftRemoveNode();
  TestRemoveNodesByRRPG();
}
