#include "search_factory.h"

#include "search/dehc.h"
#include "search/gbfs.h"
#include "search/kgbfs.h"
#include "search/lazy_gbfs.h"
#include "search/mrw13.h"
#include "search/orbit_gbfs.h"
#include "search/symmetry_breaking_gbfs.h"
#include "search/simhdgbfs.h"
#include "search/simhdgbfs1.h"
#include "multithread_search/multi_gbfs.h"

namespace pplanner {

std::unique_ptr<Search> SearchFactory(std::shared_ptr<const SASPlus> problem,
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

  if (search.get() == "kgbfs")
    return std::unique_ptr<KGBFS>(new KGBFS(problem, option.get()));

  if (search.get() == "ogbfs")
    return std::unique_ptr<OrbitGBFS>(new OrbitGBFS(problem, option.get()));

  if (search.get() == "sbgbfs")
    return std::unique_ptr<SBGBFS>(new SBGBFS(problem, option.get()));

  if (search.get() == "dehc")
    return std::unique_ptr<DEHC>(new DEHC(problem, option.get()));

  if (search.get() == "simhdgbfs")
    return std::unique_ptr<SIMHDGBFS>(new SIMHDGBFS(problem, option.get()));

  if (search.get() == "simhdgbfs1")
    return std::unique_ptr<SIMHDGBFS1>(new SIMHDGBFS1(problem, option.get()));

  if (search.get() == "multi_gbfs")
    return std::unique_ptr<MultiGBFS>(new MultiGBFS(problem, option.get()));

  throw std::runtime_error("No such search algorithm.");
}


} // namespace pplanner
