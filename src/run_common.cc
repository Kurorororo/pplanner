#include "run_common.h"

#include <fstream>
#include <iostream>

namespace rwls {

void WritePlan(const Domain &domain, const std::vector<int> &plan) {
  std::ofstream sas_plan;
  sas_plan.open("sas_plan", std::ios::out);
  int cost = 0;

  for (auto a : plan) {
    std::cout << domain.names[a] << "(" << domain.costs[a] << ")"
              << std::endl;
    sas_plan << "(" << domain.names[a] << ")" << std::endl;
    cost += domain.costs[a];
  }

  if (domain.metric == 0)
    sas_plan << "; cost = " << cost << " (unit cost)" << std::endl;
  else
    sas_plan << "; cost = " << cost << " (general cost)" << std::endl;

  int step = static_cast<int>(plan.size());

  std::cout << "Plan length: " << step << " step(s)" << std::endl;
  std::cout << "Plan cost: " << cost << std::endl;
}

void PrintStatistics(const SearchStatistics &stat) {
  std::cout << "Expanded " << stat.expanded << " state(s)" << std::endl;
  std::cout << "Evaluated " << stat.evaluated << " state(s)" << std::endl;
  std::cout << "Generated " << stat.generated << " state(s)" << std::endl;
  std::cout << "Dead ends " << stat.deadend << " state(s)" << std::endl;
  std::cout << "Search time: " << stat.search_time << "s" << std::endl;
  std::cout << "Solution found." << std::endl;
}

}
