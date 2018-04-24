#include "domain/parser.h"

#include <iostream>
#include <string>
#include <vector>

#include "domain/domain.h"
#include "domain/var_value.h"

using namespace rwls;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: sas_parser <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  Domain domain;
  Parse(filename, &domain);

  std::cout << std::endl << "variables size " << domain.variables_size;
  std::cout << std::endl << "variable" << std::endl;
  for (size_t i=0; i<domain.variables_size; ++i)
    std::cout << "var" << i << " < " << domain.dom[i] << std::endl;

  std::cout << std::endl << "fact offset" << std::endl;
  for (size_t i=0; i<domain.variables_size; ++i)
    std::cout << "var" << i << "=0: " << domain.fact_offset[i] << std::endl;
  std::cout << "fact size " << static_cast<int>(domain.fact_size) << std::endl;

  for (int i=0, n=static_cast<int>(domain.variables_size); i<n; ++i) {
    for (int j=0; j<domain.dom[i]; ++j) {
      int f = ToFact(domain.fact_offset, i, j);
      std::cout << "fact " << f << " predicate ";
      std::cout << domain.fact_to_predicate[f] << std::endl;
    }
  }

  std::cout << std::endl << "initial" << std::endl;
  for (size_t i=0; i<domain.initial.size(); ++i)
    std::cout << "var" << i << "=" << domain.initial[i] << std::endl;

  std::cout << std::endl << "mutex_groups" << std::endl;
  for (size_t i=0; i<domain.mutex_groups.size(); ++i) {
    std::cout << "id: " << i << std::endl;
    for (auto v : domain.mutex_groups[i]) {
      int var, value;
      DecodeVarValue(v, &var, &value);
      std::cout << "var" << var << "=" << value << ", ";
    }
    std::cout << std::endl;
  }

  std::cout << std::endl << "goal" << std::endl;
  for (auto v : domain.goal) {
    int var, value;
    DecodeVarValue(v, &var, &value);
    std::cout << "var" << var << " = " << value << std::endl;
  }

  std::cout << std::endl << "operators size " << domain.action_size;
  std::cout << std::endl <<  "operators" << std::endl << std::endl;
  for (size_t i=0; i<domain.names.size(); ++i) {
    std::cout << "id=" << i << std::endl;
    std::cout << domain.names[i] << std::endl;
    std::cout << "cost = " << domain.costs[i] << std::endl;
    std::cout << "precondition" << std::endl;
    for (auto v : domain.preconditions[i]) {
      int var, value;
      DecodeVarValue(v, &var, &value);
      std::cout << "var" << var << "=" << value << ", ";
    }
    std::cout << std::endl;
    std::cout << "effect" << std::endl;
    for (auto v : domain.effects[i]) {
      int var, value;
      DecodeVarValue(v, &var, &value);
      std::cout << "var" << var <<  "=" << value << ", ";
    }
    std::cout << std::endl << std::endl;
  }
}
