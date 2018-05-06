#include "search_factory.h"

namespace pplanner {

std::unique_ptr<Search> SearchFactory(const std::shared_ptr<SASPlus> &problem,
                                      const boost::property_tree::ptree &pt) {
  auto search = pt.get_optional<std::string>("search");
  if (!search) throw std::runtime_error("Parameter search is needed.");

  auto option = pt.get_child_optional("option");
  if (!option) throw std::runtime_error("Parameter option is needed.");

  if (search.get() == "mrw")
    return std::unique_ptr<Mrw13>(new Mrw13(problem, option.get()));

  if (search.get() == "gbfs")
    return std::unique_ptr<GBFS>(new GBFS(problem, option.get()));

  throw std::runtime_error("No such search algorithm.");
}


} // namespace pplanner
