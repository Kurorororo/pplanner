#include "utils/file_utils.h"

#include <fstream>
#include <iostream>

namespace pplanner {

using std::queue;
using std::string;

queue<string> FileToLines(const string &filename) {
  std::ifstream input;
  input.open(filename, std::ios::in);
  std::string buffer;
  std::queue<std::string> lines;

  while (std::getline(input, buffer)) lines.push(buffer);

  input.close();

  return lines;
}

void WritePlan(const SASPlus &sas, const std::vector<int> &plan) {
  std::ofstream sas_plan;
  sas_plan.open("sas_plan", std::ios::out);
  int total_cost = 0;

  for (auto a : plan) {
    auto name = sas.ActionName(a);
    int cost = sas.ActionCost(a);
    std::cout << name << "(" << cost << ")" << std::endl;
    sas_plan << "(" << name << ")" << std::endl;
    total_cost += cost;
  }

  if (sas.metric() == 0)
    sas_plan << "; cost = " << total_cost << " (unit cost)" << std::endl;
  else
    sas_plan << "; cost = " << total_cost << " (general cost)" << std::endl;

  int step = static_cast<int>(plan.size());

  std::cout << "Plan length: " << step << " step(s)" << std::endl;
  std::cout << "Plan cost: " << total_cost << std::endl;
}

}  // namespace pplanner
