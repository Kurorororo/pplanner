#include "search_factory.h"

#include "search/gbfs.h"
#include "search/lazy_gbfs.h"
#include "search/mrw13.h"

namespace pplanner {

std::unique_ptr<Search> SearchFactory(const std::shared_ptr<SASPlus> &problem,
                                      const boost::property_tree::ptree &pt,
                                      int max_expansion) {
  auto search = pt.get_optional<std::string>("search");
  if (!search) throw std::runtime_error("Parameter search is needed.");

  auto option = pt.get_child_optional("option");
  if (!option) throw std::runtime_error("Parameter option is needed.");

  if (search.get() == "mrw")
    return std::unique_ptr<Mrw13>(
        new Mrw13(problem, option.get(), max_expansion));

  if (search.get() == "gbfs")
    return std::unique_ptr<GBFS>(new GBFS(problem, option.get()));

  if (search.get() == "lazy_gbfs")
    return std::unique_ptr<LazyGBFS>(new LazyGBFS(problem, option.get()));

  throw std::runtime_error("No such search algorithm.");
}


} // namespace pplanner
