#include <chrono>
#include <iostream>

#include <mpi.h>

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "sas_plus.h"
#include "mpi_search_factory.h"
#include "postprocess/action_elimination.h"
#include "utils/file_utils.h"

using namespace pplanner;

bool Run(std::shared_ptr<const SASPlus> sas,
         const boost::property_tree::ptree &pt,
         bool postprocess) {
  auto chrono_start = std::chrono::system_clock::now();
  auto search = MpiSearchFactory(sas, pt);
  auto result = search->Plan();
  auto chrono_end = std::chrono::system_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
     chrono_end - chrono_start).count();
  auto search_time = static_cast<double>(ns) / 1e9;

  int rank;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  if (rank == 0) {
    if (postprocess) result = ActionElimination(*sas, result);


    if (result.empty() || result[0] > 0)
      WritePlan(*sas, result);

    search->DumpStatistics();
    std::cout << "Search time: " << search_time << "s" << std::endl;
  }

  if (!result.empty() && result[0] == -1) {
    if (rank == 0) std::cerr << "faild to solve instance" << std::endl;
    return false;
  }

  return true;
}

int main(int argc, char *argv[]) {
  MPI_Init(&argc, &argv);

  namespace po = boost::program_options;
  po::options_description opt("Options");
  opt.add_options()
    ("help,h", "help")
    ("file,f", po::value<std::string>(), "input sas+ file name")
    ("config,c", po::value<std::string>(), "run config file name")
    ("max_expansion,m", po::value<int>(), "limit of node expansion")
    ("postprocess,p", "postprocess plan");
  po::variables_map vm;

  try {
    po::store(po::parse_command_line(argc, argv, opt), vm);
  } catch(const po::error_with_option_name& e) {
    std::cout << e.what() << std::endl;
  }

  po::notify(vm);

  if (vm.count("help") || !vm.count("file") || !vm.count("config")) {
    std::cout << opt << std::endl;
    exit(0);
  }

  auto filename = vm["file"].as<std::string>();
  auto lines = FileToLines(filename);
  auto sas = std::make_shared<SASPlus>();
  sas->InitFromLines(lines);

  auto config_filename = vm["config"].as<std::string>();
  boost::property_tree::ptree pt;
  read_json(config_filename, pt);
  auto child = pt.get_child_optional("config");
  if (!child) throw std::runtime_error("Invalid config file.");
  bool postprocess = vm.count("postprocess");
  bool result = Run(sas, child.get(), postprocess);

  MPI_Finalize();

  if (!result) exit(1);
}
