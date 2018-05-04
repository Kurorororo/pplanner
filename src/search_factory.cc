#include "search_factory.h"

namespace pplanner {

std::unique_ptr<Search> SearchFactory(const std::shared_ptr<SASPlus> &problem,
                                      const boost::property_tree::ptree &pt) {
  auto search = pt.get_optional<std::string>("search");
  if (!search) throw std::runtime_error("Parameter search is needed.");

  auto options = pt.get_child_optional("options");
  if (!options) throw std::runtime_error("Parameter options is needed.");

  if (search.get() == "mrw")
    return std::unique_ptr<Mrw13>(new Mrw13(problem, options.get()));

  throw std::runtime_error("No such search algorithm.");
}


} // namespace pplanner
