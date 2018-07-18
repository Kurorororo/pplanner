#include "mpi_search_factory.h"

#include "search/hdgbfs.h"
#include "search/pddsgbfs.h"

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

  if (search.get() == "pddsgbfs")
    return std::unique_ptr<PDDSGBFS>(new PDDSGBFS(problem, option.get()));

  throw std::runtime_error("No such search algorithm.");
}


} // namespace pplanner