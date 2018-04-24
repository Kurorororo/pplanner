#include "search/hdwastar.h"

#include <chrono>
#include <fstream>
#include <iostream>

#include <boost/program_options.hpp>

#include <mpi.h>

#include "domain/domain.h"
#include "domain/parser.h"
#include "heuristic/additive.h"
#include "heuristic/ff.h"
#include "heuristic/ff_add.h"

using namespace rwls;

int main(int argc, char *argv[]) {
  MPI_Init(&argc, &argv);

  namespace po = boost::program_options;
  po::options_description opt("Options");
  opt.add_options()
    ("help,h", "help")
    ("file,f", po::value<std::string>(), "input sas+ file name")
    ("heuristic,e", po::value<std::string>(), "heuristic function")
    ("weight,w", po::value<int>(), "weight of heuristic value. default: 1");
  po::variables_map vm;

  try {
    po::store(po::parse_command_line(argc, argv, opt), vm);
  } catch(const po::error_with_option_name& e) {
    std::cout << e.what() << std::endl;
  }

  po::notify(vm);

  if (vm.count("help") || !vm.count("file") || !vm.count("heuristic")) {
    std::cout << opt << std::endl;
    exit(0);
  }

  std::string filename = vm["file"].as<std::string>();
  std::string heuristic_name = vm["heuristic"].as<std::string>();

  int weight = 1;

  if (vm.count("weight"))
    weight = vm["weight"].as<int>();

  Domain domain;
  Parse(filename, &domain);
  auto chrono_start = std::chrono::system_clock::now();

  std::vector<int> result;

  int expanded = 0;
  int evaluated = 0;
  int generated = 0;
  int deadend = 0;

  if (heuristic_name == "ff") {
    HDWAstar<FF> solver(weight, domain);
    result = solver();
    expanded = solver.expanded;
    evaluated = solver.evaluated;
    generated = solver.generated;
    deadend = solver.deadend;
  } else if (heuristic_name == "fa") {
    HDWAstar<FFAdd> solver(weight, domain);
    result = solver();
    expanded = solver.expanded;
    evaluated = solver.evaluated;
    generated = solver.generated;
    deadend = solver.deadend;
  } else if (heuristic_name == "add") {
    HDWAstar<Additive> solver(weight, domain);
    result = solver();
    expanded = solver.expanded;
    evaluated = solver.evaluated;
    generated = solver.generated;
    deadend = solver.deadend;
  } else {
    std::cout << opt << std::endl;
    exit(0);
  }

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
  std::cout << "Expanded " << expanded << " state(s)" << std::endl;
  std::cout << "Evaluated " << evaluated << " state(s)" << std::endl;
  std::cout << "Generated " << generated << " state(s)" << std::endl;
  std::cout << "Dead ends " << deadend << " state(s)" << std::endl;
  std::cout << "Search time: " << search_time << "s" << std::endl;
  std::cout << "Solution found." << std::endl;

  MPI_Finalize();
}
