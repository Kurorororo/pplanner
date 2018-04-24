#include "search/lazy_gbfs.h"
#include "search/preferred_lazy_gbfs.h"

#include <chrono>
#include <iostream>

#include <boost/program_options.hpp>

#include <mpi.h>

#include "run_common.h"
#include "domain/domain.h"
#include "domain/parser.h"
#include "heuristic/additive.h"
#include "heuristic/ff.h"
#include "heuristic/ff_s.h"
#include "heuristic/ff_add.h"
#include "heuristic/landmark_count.h"

using namespace rwls;

int main(int argc, char *argv[]) {
  namespace po = boost::program_options;
  po::options_description opt("Options");
  opt.add_options()
    ("help,h", "help")
    ("file,f", po::value<std::string>(), "input sas+ file name")
    ("heuristic,e", po::value<std::string>(), "heuristic function")
    ("preferred,p", "use preferred operators")
    ("n_boost,n", po::value<int>(), "boost preferred open list. default: 0")
    ("initial_boost,i", "boost initial");
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
  bool preferred = vm.count("preferred");
  int n_boost = 0;
  if (vm.count("n_boost")) n_boost = vm["n_boost"].as<int>();
  bool initial_boost = vm.count("initial_boost");

  Domain domain;
  Parse(filename, &domain);
  auto chrono_start = std::chrono::system_clock::now();

  std::vector<int> result;
  SearchStatistics stat;

  if (heuristic_name == "ff") {
    if (preferred) {
      PreferredLazyGBFS<FF> solver;
      result = solver(domain, n_boost, initial_boost);
      SetStatistics(solver, stat);
    } else {
      LazyGBFS<FF> solver;
      result = solver(domain);
      SetStatistics(solver, stat);
    }
  } else if (heuristic_name == "fs") {
    if (preferred) {
      PreferredLazyGBFS<FFS> solver;
      result = solver(domain, n_boost, initial_boost);
      SetStatistics(solver, stat);
    } else {
      LazyGBFS<FFS> solver;
      result = solver(domain);
      SetStatistics(solver, stat);
    }
  } else if (heuristic_name == "fa") {
    if (preferred) {
      PreferredLazyGBFS<FFAdd> solver;
      result = solver(domain, n_boost, initial_boost);
      SetStatistics(solver, stat);
    } else {
      LazyGBFS<FFAdd> solver;
      result = solver(domain);
      SetStatistics(solver, stat);
    }
  } else if (heuristic_name == "add") {
    LazyGBFS<Additive> solver;
    result = solver(domain);
    SetStatistics(solver, stat);
  } else {
    std::cout << opt << std::endl;
    exit(0);
  }

  auto chrono_end = std::chrono::system_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
     chrono_end - chrono_start).count();
  stat.search_time = static_cast<double>(ns) / 1e9;

  WritePlan(domain, result);
  PrintStatistics(stat);
}
