#include "domain/domain.h"

#include <cassert>

#include <iostream>

using namespace rwls;

void TestToFact() {
  std::vector<int> fact_offset{0, 2, 5, 10};
  assert(1 == ToFact(fact_offset, 0, 1));
  assert(4 == ToFact(fact_offset, 1, 2));
  assert(8 == ToFact(fact_offset, 2, 3));
  assert(14 == ToFact(fact_offset, 3, 4));

  std::cout << "passed ToFact" << std::endl;
}

void TestToFactVarValue() {
  std::vector<int> fact_offset{0, 2, 5, 10};
  VarValue v;
  EncodeVarValue(0, 1, &v);
  assert(1 == ToFact(fact_offset, v));
  EncodeVarValue(1, 2, &v);
  assert(4 == ToFact(fact_offset, v));
  EncodeVarValue(2, 3, &v);
  assert(8 == ToFact(fact_offset, v));
  EncodeVarValue(3, 4, &v);
  assert(14 == ToFact(fact_offset, v));

  std::cout << "passed ToFact VarValue" << std::endl;
}

void TestToFacts() {
  Domain domain;
  domain.fact_size = 7;
  domain.variables_size = 3;
  domain.fact_offset = std::vector<int>{0, 2, 5};
  /* var1={0, 1}, var2={0, 1, 2}, var3={0, 1} */
  State state{1, 2, 0};
  /* 0100110 */
  std::vector<bool> proposition{false, true, false, false, true, true, false};
  assert(proposition == ToFacts(domain, state));

  std::cout << "passed ToFacts" << std::endl;
}

void TestHammingDistance() {
  std::vector<bool> a{false, true, false, false, true, true, false};
  std::vector<bool> b{false, true, false, false, true, true, false};

  assert(0 == HammingDistance(a, b));
  b[0] = true;
  assert(1 == HammingDistance(a, b));
  b[4] = false;
  assert(2 == HammingDistance(a, b));

  std::cout << "passed HammingDistance" << std::endl;
}

int main() {
  TestToFact();
  TestToFactVarValue();
  TestToFacts();
  TestHammingDistance();
}
