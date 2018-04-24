#include "heuristic/simplified_graphplan.h"

#include <algorithm>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "domain/parser.h"
#include "domain/state.h"
#include "domain/var_value.h"

using namespace rwls;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: sas_simplified_graphplan <filename>" << std::endl;
    exit(1);
  }

  std::string filename = argv[1];
  Domain domain;
  Parse(filename, &domain);
  RelaxedDomain r_domain;
  InitializeRelaxedDomain(domain, &r_domain);
  PlanningGraph graph;
  InitializeGraph(r_domain, &graph);
  std::unordered_set<int> preferred;
  auto initial = domain.initial;
  std::vector<int> initial_facts;
  StateToFactSet(initial, domain, initial_facts);
  auto result = Graphplan(initial_facts, r_domain, &graph, preferred);
  int result_cost = GraphplanCost(initial_facts, r_domain, &graph, preferred);
  std::cout << "plan" << std::endl;
  int cost = 0;

  for (auto o : result) {
    if (o == -1) {
      std::cerr << "faild to solve problem" << std::endl;
      exit(0);
    }

    cost += domain.costs[o];
    std::cout << "number " << o << " " << domain.names[o] << std::endl;
  }

  std::cout << std::endl;

  std::cout << "plan step: " << result.size() << std::endl;
  assert(cost == result_cost);
  std::cout << "plan cost: " << cost << std::endl;
  std::cout << std::endl;

  std::cout << "preferred" << std::endl;

  for (auto o : preferred)
    std::cout << domain.names[o] << std::endl;

  std::cout << std::endl;

  std::cout << "test" << std::endl;
  std::vector<int> facts(domain.fact_size, 0);
  std::cout << "ini: ";

  for (size_t i=0; i<initial.size(); ++i) {
    std::cout << "fact" << domain.fact_offset[i] + initial[i] << "|";
    std::cout << "var" << i << "=" << initial[i] << ", ";
    facts[domain.fact_offset[i]+initial[i]] = 1;
  }

  std::cout << std::endl << std::endl;

  for (auto o : result) {
    std::cout << domain.names[o] << std::endl;
    std::cout << "number " << o << std::endl;
    std::cout << "pre: ";

    for (auto f : domain.preconditions[o]) {
      int var, value;
      DecodeVarValue(f, &var, &value);
      std::cout << "fact" << domain.fact_offset[var] + value << "|";
      std::cout << "var" << var << "=" << value << ", ";

      if (facts[domain.fact_offset[var]+value] == 0) {
        std::cout << std::endl << std::endl;
        std::cerr << "fact" << domain.fact_offset[var] + value << "|var"
                  << var << "=" << value << " is not satisfied for action "
                  << o  << " " << domain.names[o] << std::endl;
        exit(1);
      }
    }

    std::cout << std::endl << "eff: ";

    for (auto f : domain.effects[o]) {
      int var, value;
      DecodeVarValue(f, &var, &value);
      std::cout << "fact" << domain.fact_offset[var] + value << "|";
      std::cout << "var" << var << "=" << value << ", ";
      facts[domain.fact_offset[var]+value] = 1;
    }

    std::cout << std::endl << std::endl;
  }

  for (auto g : domain.goal) {
    int var, value;
    DecodeVarValue(g, &var, &value);

    if (facts[domain.fact_offset[var]+value] == 0) {
      std::cerr << "goal var" << var << "=" << value << " is not satisfied"
                << std::endl;
      exit(1);
    }
  }

  std::cout << "OK!" << std::endl;
}
