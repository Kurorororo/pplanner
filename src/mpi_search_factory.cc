#include "mpi_search_factory.h"

#include "mpi_search/hddehc.h"
#include "mpi_search/hdgbfs.h"
#include "mpi_search/hdgbfs1.h"
#include "mpi_search/symmetry_breaking_hdgbfs.h"

namespace pplanner {

std::unique_ptr<Search> MpiSearchFactory(
    std::shared_ptr<const SASPlus> &problem,
    const boost::property_tree::ptree &pt) {
  auto search = pt.get_optional<std::string>("search");
  if (!search) throw std::runtime_error("Parameter search is needed.");

  auto option = pt.get_child_optional("option");
  if (!option) throw std::runtime_error("Parameter option is needed.");

  if (search.get() == "hdgbfs")
    return std::unique_ptr<HDGBFS>(new HDGBFS(problem, option.get()));

  if (search.get() == "sbhdgbfs")
    return std::unique_ptr<SBHDGBFS>(new SBHDGBFS(problem, option.get()));

  if (search.get() == "hddehc")
    return std::unique_ptr<HDDEHC>(new HDDEHC(problem, option.get()));

  if (search.get() == "hdgbfs1")
    return std::unique_ptr<HDGBFS1>(new HDGBFS1(problem, option.get()));

  throw std::runtime_error("No such search algorithm.");
}

}  // namespace pplanner
