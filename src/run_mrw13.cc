#include "random_walk/mrw13.h"
#include "random_walk/mrw13_lmcount.h"
#include "random_walk/mrw13_multi.h"
#include "random_walk/mrw13_multi_lmcount.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include <boost/program_options.hpp>

#include "domain/domain.h"
#include "domain/parser.h"
#include "heuristic/ff.h"
#include "heuristic/ff_add.h"

int generated = 0;
int expanded = 0;
int evaluated = 0;
int deadend = 0;
int evaluations = 0;

int main(int argc, char *argv[]) {
  namespace po = boost::program_options;
  po::options_description opt("Options");
  opt.add_options()
    ("help,h", "help")
    ("file,f", po::value<std::string>(), "input sas+ file name")
    ("heuristic,e", po::value<std::string>(), "heuristic function")
    ("fix", "use fixed length random walk")
    ("uniform,u", "use uniform sampling")
    ("multi,m", po::value<int>(), "number of random walks from a state");

  po::variables_map vm;

  try {
    po::store(po::parse_command_line(argc, argv, opt), vm);
  } catch(const po::error_with_option_name& e) {
    std::cout << e.what() << std::endl;
  }

  po::notify(vm);

  if (vm.count("help") || !vm.count("file")) {
    std::cout << opt << std::endl;
    exit(0);
  }

  std::string filename = vm["file"].as<std::string>();
  std::string heuristic = vm["heuristic"].as<std::string>();

  int multi = 0;

  if (vm.count("multi")) multi = vm["multi"].as<int>();

  bool uniform = false;
  bool fix = false;

  if (vm.count("uniform")) uniform = true;
  if (vm.count("fix")) fix = true;

  rwls::Domain domain;
  rwls::Parse(filename, &domain);
  std::vector<int> result;
  auto chrono_start = std::chrono::system_clock::now();

  if (multi == 0) {
    if (heuristic == "lmc") {
      rwls::Mrw13Lmcount mrw(domain);
      result = mrw(fix);
    } else if (heuristic == "ff") {
      rwls::Mrw13<rwls::FF> mrw(domain, uniform);
      result = mrw(fix);
    } else if (heuristic == "fa") {
      rwls::Mrw13<rwls::FFAdd> mrw(domain, uniform);
      result = mrw(fix);
    } else {
      std::cout << "not supported heuristic function" << std::endl;
      exit(0);
    }
  } else {
    if (heuristic == "lmc") {
      rwls::Mrw13MultiLmcount mrw(domain);
      result = mrw(multi);
    } else if (heuristic == "fa") {
      rwls::Mrw13Multi<rwls::FFAdd> mrw(domain, uniform);
      result = mrw(multi);
    } else {
      std::cout << "not supported heuristic function" << std::endl;
      exit(0);
    }
  }

  if (result[0] == -1) {
    std::cout << "faild to solve problem" << std::endl;
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
  std::cout << "Evaluations " << evaluations << std::endl;
  std::cout << "Generated " << generated << " state(s)" << std::endl;
  std::cout << "Dead ends " << deadend << " state(s)" << std::endl;
  std::cout << "Search time: " << search_time << "s" << std::endl;
  std::cout << "Solution found." << std::endl;
}
