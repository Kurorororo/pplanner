#include "trie/trie.h"

#include <cassert>

#include <iostream>
#include <vector>

#include "domain/domain.h"
#include "domain/state.h"
#include "domain/var_value.h"

using namespace rwls;

void TestInsertToTable() {
  std::vector<int> fact_offset{0, 2, 5, 10};
  Domain domain;
  domain.action_size = 2;
  domain.variables_size = 4;
  domain.fact_size = 12;
  domain.fact_offset = fact_offset;
  TrieTable table;
  InitializeTable(domain, &table);
  std::vector<VarValue> precondition(4);
  EncodeVarValue(0, 1, &precondition[0]);
  EncodeVarValue(1, 2, &precondition[1]);
  EncodeVarValue(2, 4, &precondition[2]);
  EncodeVarValue(3, 1, &precondition[3]);
  InsertToTable(domain, 0, precondition, &table);
  FinalizeTable(&table);

  std::cout << "passed InsertToTable" << std::endl;
}

void TestConstructTable() {
  std::vector<int> fact_offset{0, 2, 5, 10};
  std::vector< std::vector<VarValue> > preconditions(2);
  VarValue var_value;
  EncodeVarValue(0, 1, &var_value);
  preconditions[0].push_back(var_value);
  EncodeVarValue(2, 4, &var_value);
  preconditions[0].push_back(var_value);
  EncodeVarValue(3, 1, &var_value);
  preconditions[0].push_back(var_value);
  EncodeVarValue(0, 1, &var_value);
  preconditions[1].push_back(var_value);
  EncodeVarValue(1, 2, &var_value);
  preconditions[1].push_back(var_value);
  EncodeVarValue(2, 4, &var_value);
  preconditions[1].push_back(var_value);
  EncodeVarValue(3, 1, &var_value);
  preconditions[1].push_back(var_value);
  Domain domain;
  domain.action_size = 2;
  domain.variables_size = 4;
  domain.fact_size = 12;
  domain.preconditions = preconditions;
  domain.fact_offset = fact_offset;
  auto table = ConstructTable(domain);
  FinalizeTable(&table);

  std::cout << "passed ConstructTable" << std::endl;
}

void TestFindFromTable() {
  std::vector<int> fact_offset{0, 2, 5, 10};
  Domain domain;
  domain.action_size = 2;
  domain.variables_size = 4;
  domain.fact_size = 12;
  domain.fact_offset = fact_offset;
  TrieTable table;
  InitializeTable(domain, &table);
  std::vector<VarValue> precondition(3);
  EncodeVarValue(0, 1, &precondition[0]);
  EncodeVarValue(2, 4, &precondition[1]);
  EncodeVarValue(3, 1, &precondition[2]);
  InsertToTable(domain, 0, precondition, &table);

  State variables{1, 2, 4, 1};
  auto result = FindFromTable(table, domain, variables);
  assert(1 == result.size());
  assert(0 == result[0]);
  variables[1] = 0;
  result = FindFromTable(table, domain, variables);
  assert(1 == result.size());
  assert(0 == result[0]);
  VarValue var_value;
  EncodeVarValue(1, 0, &var_value);
  precondition.push_back(var_value);
  InsertToTable(domain, 1, precondition, &table);
  result = FindFromTable(table, domain, variables);
  assert(2 == result.size());
  if (result[0] == 0) {
    assert(1 == result[1]);
  } else {
    assert(1 == result[0]);
    assert(0 == result[1]);
  }
  variables[1] = 2;
  result = FindFromTable(table, domain, variables);
  assert(1 == result.size());
  assert(0 == result[0]);
  variables[2] = 2;
  result = FindFromTable(table, domain, variables);
  assert(0 == result.size());

  FinalizeTable(&table);

  std::cout << "passed FindFromTable" << std::endl;
}

void TestVoidFindFromTable() {
  std::vector<int> fact_offset{0, 2, 5, 10};
  Domain domain;
  domain.action_size = 2;
  domain.variables_size = 4;
  domain.fact_size = 12;
  domain.fact_offset = fact_offset;
  TrieTable table;
  InitializeTable(domain, &table);
  std::vector<VarValue> precondition(3);
  EncodeVarValue(0, 1, &precondition[0]);
  EncodeVarValue(2, 4, &precondition[1]);
  EncodeVarValue(3, 1, &precondition[2]);
  InsertToTable(domain, 0, precondition, &table);

  State variables{1, 2, 4, 1};
  std::vector<int> result;
  FindFromTable(table, domain, variables, result);
  assert(1 == result.size());
  assert(0 == result[0]);
  variables[1] = 0;
  FindFromTable(table, domain, variables, result);
  assert(1 == result.size());
  assert(0 == result[0]);
  VarValue var_value;
  EncodeVarValue(1, 0, &var_value);
  precondition.push_back(var_value);
  InsertToTable(domain, 1, precondition, &table);
  FindFromTable(table, domain, variables, result);
  assert(2 == result.size());
  if (result[0] == 0) {
    assert(1 == result[1]);
  } else {
    assert(1 == result[0]);
    assert(0 == result[1]);
  }
  variables[1] = 2;
  FindFromTable(table, domain, variables, result);
  assert(1 == result.size());
  assert(0 == result[0]);
  variables[2] = 2;
  FindFromTable(table, domain, variables, result);
  assert(0 == result.size());

  FinalizeTable(&table);

  std::cout << "passed FindFromTable" << std::endl;
}

int main() {
  TestInsertToTable();
  TestConstructTable();
  TestFindFromTable();
  TestVoidFindFromTable();
}
