#include "domain/var_value.h"

#include <cassert>

#include <iostream>

using namespace rwls;

void TestVarValue() {
  VarValue v;
  EncodeVarValue(3, 2, &v);
  int var, value;
  DecodeVarValue(v, &var, &value);
  assert(3 == var);
  assert(2 == value);
  EncodeVarValue(22, 1000, &v);
  DecodeVarValue(v, &var, &value);
  assert(22 == var);
  assert(1000 == value);

  std::cout << "passed VarValue" << std::endl;
}

void TestGetVar() {
  VarValue v;
  EncodeVarValue(3, 2, &v);
  assert(3 == GetVar(v));
  EncodeVarValue(22, 1000, &v);
  assert(22 == GetVar(v));

  std::cout << "passed GetVar" << std::endl;
}

void TestGetValue() {
  VarValue v;
  EncodeVarValue(3, 2, &v);
  assert(2 == GetValue(v));
  EncodeVarValue(22, 1000, &v);
  assert(1000 == GetValue(v));

  std::cout << "passed GetValue" << std::endl;
}

int main() {
  TestVarValue();
  TestGetVar();
  TestGetValue();
}
