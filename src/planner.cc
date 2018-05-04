#include <chrono>
#include <iostream>

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "sas_plus.h"
#include "search_factory.h"
#include "utils/file_utils.h"

using namespace pplanner;

void WritePlan(const SASPlus &sas, const std::vector<int> &plan) {
  std::ofstream sas_plan;
  sas_plan.open("sas_plan", std::ios::out);
  int total_cost = 0;

  for (auto a : plan) {
    auto name = sas.ActionName(a);
    int cost = sas.ActionCost(a);
    std::cout << name  << "(" << cost << ")" << std::endl;
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

int main(int argc, char *argv[]) {
  namespace po = boost::program_options;
  po::options_description opt("Options");
  opt.add_options()
    ("help,h", "help")
    ("file,f", po::value<std::string>(), "input sas+ file name")
    ("property,p", po::value<std::string>(), "run property file name");
  po::variables_map vm;

  try {
    po::store(po::parse_command_line(argc, argv, opt), vm);
  } catch(const po::error_with_option_name& e) {
    std::cout << e.what() << std::endl;
  }

  po::notify(vm);

  if (vm.count("help") || !vm.count("file") || !vm.count("property")) {
    std::cout << opt << std::endl;
    exit(0);
  }

  auto filename = vm["file"].as<std::string>();
  auto lines = FileToLines(filename);
  auto sas = std::make_shared<SASPlus>();
  sas->InitFromLines(lines);

  auto property_filename = vm["property"].as<std::string>();
  boost::property_tree::ptree pt;
  read_json(property_filename, pt);
  auto child = pt.get_child_optional("config");
  if (!child) throw std::runtime_error("Invalid property file.");

  auto chrono_start = std::chrono::system_clock::now();
  auto search = SearchFactory(sas, child.get());
  auto result = search->Plan();
  auto chrono_end = std::chrono::system_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
     chrono_end - chrono_start).count();
  auto search_time = static_cast<double>(ns) / 1e9;

  WritePlan(*sas, result);
  search->DumpStatistics();

  std::cout << "Search time: " << search_time << "s" << std::endl;
}
