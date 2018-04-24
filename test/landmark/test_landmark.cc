#include "landmark/landmark.h"

#include <iostream>

#include "domain/var_value.h"

using namespace rwls;

void TestConstructor() {
  Landmark landmark;
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark2(v);

  std::cout << "passed Constructer" << std::endl;
}

void TestIsEmpty() {
  Landmark landmark;
  assert(landmark.IsEmpty());

  std::cout << "passed IsEmpty" << std::endl;
}

void TestGetSize() {
  Landmark landmark;
  assert(0 == landmark.GetSize());
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark2(v);
  assert(1 == landmark2.GetSize());

  std::cout << "passed GetSize" << std::endl;
}

void TestClear() {
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  landmark.Clear();
  assert(landmark.IsEmpty());

  std::cout << "passed Clear" << std::endl;
}

void TestGetVarValue() {
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  int var, value;
  DecodeVarValue(landmark.GetVarValue(), &var, &value);
  assert(0 == var);
  assert(2 == value);

  std::cout << "passed GetVarValue" << std::endl;
}

void TestAddVarValue() {
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  EncodeVarValue(0, 4, &v);
  landmark.AddVarValue(v);
  assert(2 == landmark.GetSize());

  std::cout << "passed AddVarValue" << std::endl;
}

void TestIsFact() {
  Landmark landmark;
  assert(!landmark.IsFact());
  VarValue v;
  EncodeVarValue(0, 2, &v);
  landmark.AddVarValue(v);
  assert(landmark.IsFact());
  EncodeVarValue(0, 4, &v);
  landmark.AddVarValue(v);
  assert(!landmark.IsFact());

  std::cout << "passed IsFact" << std::endl;
}

void TestGetVarValues() {
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  EncodeVarValue(1, 4, &v);
  landmark.AddVarValue(v);
  int i = 0;
  for (auto v : landmark.GetVarValues()) {
    int var, value;
    if (i == 0) {
      DecodeVarValue(v, &var, &value);
      assert(0 == var);
      assert(2 == value);
      ++i;
    } else {
      DecodeVarValue(v, &var, &value);
      assert(1 == var);
      assert(4 == value);
    }
  }

  std::cout << "passed GetVarValues" << std::endl;
}

void TestEqual() {
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  Landmark landmark2(v);
  assert(landmark == landmark2);

  std::cout << "passed ==" << std::endl;
}

void TestNotEqual() {
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  EncodeVarValue(0, 1, &v);
  Landmark landmark2(v);
  assert(landmark != landmark2);

  std::cout << "passed !=" << std::endl;
}

void TestCopy() {
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  Landmark landmark2;
  landmark2 = landmark;
  assert(landmark == landmark2);
  Landmark landmark3(landmark);
  assert(landmark == landmark3);

  std::cout << "passed =" << std::endl;
}

void TestHash() {
  VarValue v;
  EncodeVarValue(0, 2, &v);
  Landmark landmark(v);
  Landmark landmark2(v);
  assert(landmark.Hash() == landmark2.Hash());
  Landmark landmark3(landmark);
  assert(landmark.Hash() == landmark3.Hash());
  Landmark landmark4;
  landmark4 = landmark;
  assert(landmark.Hash() == landmark4.Hash());

  LandmarkHash l_hash;
  assert(l_hash(landmark) == l_hash(landmark2));
  assert(l_hash(landmark) == l_hash(landmark3));
  assert(l_hash(landmark) == l_hash(landmark4));

  std::cout << "passed Hash" << std::endl;
}

void TestIsImplicated() {
  Landmark landmark;
  assert(!landmark.IsImplicated(3, 2));
  VarValue v;
  EncodeVarValue(1, 4, &v);
  landmark.AddVarValue(v);
  assert(landmark.IsImplicated(1, 4));
  assert(!landmark.IsImplicated(3, 2));
  EncodeVarValue(0, 10, &v);
  landmark.AddVarValue(v);
  assert(landmark.IsImplicated(1, 4));
  assert(landmark.IsImplicated(0, 10));
  assert(!landmark.IsImplicated(3, 2));

  std::cout << "passed IsImplicated" << std::endl;
}

void TestIsImplicatedAssignment() {
  std::vector<VarValue> assignment;
  std::vector<VarValue> assignment1;
  VarValue v;
  EncodeVarValue(1, 4, &v);
  assignment.push_back(v);
  EncodeVarValue(3, 2, &v);
  assignment1.push_back(v);
  Landmark landmark;
  assert(!landmark.IsImplicated(assignment));
  EncodeVarValue(1, 4, &v);
  landmark.AddVarValue(v);
  assert(landmark.IsImplicated(assignment));
  assert(!landmark.IsImplicated(assignment1));
  EncodeVarValue(3, 2, &v);
  landmark.AddVarValue(v);
  assert(landmark.IsImplicated(assignment));
  assert(landmark.IsImplicated(assignment1));

  std::cout << "passed IsImplicated Assignment" << std::endl;
}

void TestIsImplicatedState() {
  State state{2, 4};
  Landmark landmark;
  assert(!landmark.IsImplicated(state));
  VarValue v;
  EncodeVarValue(1, 4, &v);
  landmark.AddVarValue(v);
  assert(landmark.IsImplicated(state));
  EncodeVarValue(0, 10, &v);
  landmark.AddVarValue(v);
  assert(landmark.IsImplicated(state));
  state[1] = 3;
  assert(!landmark.IsImplicated(state));

  std::cout << "passed IsImplicated State" << std::endl;
}

int main() {
  TestConstructor();
  TestIsEmpty();
  TestGetSize();
  TestClear();
  TestGetVarValue();
  TestAddVarValue();
  TestIsFact();
  TestGetVarValues();
  TestEqual();
  TestNotEqual();
  TestCopy();
  TestHash();
  TestIsImplicated();
  TestIsImplicatedAssignment();
  TestIsImplicatedState();
}
