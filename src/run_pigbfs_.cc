#include "pigbfs.h"

#include <fstream>
#include <iostream>

#include <mpi.h>

#include "domain/domain.h"
#include "domain/parser.h"
#include "heuristic/landmark_count.h"
#include "heuristic/ff.h"

using namespace rwls;

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: hdgbfs <filename>" << std::endl;
    exit(1);
  }

  MPI_Init(&argc, &argv);

  std::string filename = argv[1];
  Domain domain;
  Parse(filename, &domain);
  auto chrono_start = std::chrono::system_clock::now();
  PIGBFS<FF> solver(domain);
  auto result = solver();

  if (!result.empty()) {
    auto chrono_end = std::chrono::system_clock::now();
    std::cout << "Solution found!" << std::endl;
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
       chrono_end - chrono_start).count();
    double search_time = static_cast<double>(ns) / 1e9;
    std::cout << "Acutual search time: " << search_time << "s" << std::endl;

    std::ofstream sas_plan;
    sas_plan.open("sas_plan", std::ios::out);
    int cost = 0;
    for (auto a : result) {
      std::cout << domain.names[a] << "(" << domain.costs[a] << ")"
                << std::endl;
      sas_plan << "(" << domain.names[a] << ")" << std::endl;
      cost += domain.costs[a];
    }
    if (domain.metric == 0)
      sas_plan << "; cost = " << cost << " (unit cost)" << std::endl;
    else
      sas_plan << "; cost = " << cost << " (general cost)" << std::endl;

    int step = static_cast<int>(result.size());

    std::cout << "Plan length: " << step << " step(s)" << std::endl;
    std::cout << "Plan cost: " << cost << std::endl;
    std::cout << "Expanded " << solver.expanded << " state(s)" << std::endl;
    std::cout << "Evaluated " << solver.evaluated << " state(s)" << std::endl;
    std::cout << "Generated " << solver.generated << " state(s)" << std::endl;
    std::cout << "Dead ends " << solver.deadend << " state(s)" << std::endl;
    std::cout << "Search time: " << search_time << "s" << std::endl;
    std::cout << "Solution found." << std::endl;
  }

  MPI_Finalize();
}
