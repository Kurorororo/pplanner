#include "search_factory.h"

#include "multithread_search/kpgbfs.h"
#include "multithread_search/pgbfs.h"
#include "multithread_search/spuhf.h"
#include "search/bmrw.h"
#include "search/bts_gbfs.h"
#include "search/dehc.h"
#include "search/gbfs.h"
#include "search/kgbfs.h"
#include "search/lazy_gbfs.h"
#include "search/mrw13.h"
#include "search/orbit_gbfs.h"
#include "search/symmetry_breaking_gbfs.h"

namespace pplanner {

std::unique_ptr<Search> SearchFactory(std::shared_ptr<const SASPlus> problem,
                                      const boost::property_tree::ptree &pt,
                                      int max_expansion) {
  auto search = pt.get_optional<std::string>("search");
  if (!search) throw std::runtime_error("Parameter search is needed.");

  auto option = pt.get_child_optional("option");
  if (!option) throw std::runtime_error("Parameter option is needed.");

  if (search.get() == "mrw")
    return std::make_unique<Mrw13>(problem, option.get(), max_expansion);

  if (search.get() == "bmrw")
    return std::make_unique<BMRW>(problem, option.get());

  if (search.get() == "gbfs")
    return std::make_unique<GBFS>(problem, option.get());

  if (search.get() == "bts_gbfs")
    return std::make_unique<BTSGBFS>(problem, option.get());

  if (search.get() == "lazy_gbfs")
    return std::make_unique<LazyGBFS>(problem, option.get());

  if (search.get() == "kgbfs")
    return std::make_unique<KGBFS>(problem, option.get());

  if (search.get() == "ogbfs")
    return std::make_unique<OrbitGBFS>(problem, option.get());

  if (search.get() == "sbgbfs")
    return std::make_unique<SBGBFS>(problem, option.get());

  if (search.get() == "dehc")
    return std::make_unique<DEHC>(problem, option.get());

  if (search.get() == "kpgbfs")
    return std::make_unique<KPGBFS>(problem, option.get());

  if (search.get() == "spuhf")
    return std::make_unique<SPUHF>(problem, option.get());

  if (search.get() == "pgbfs")
    return std::make_unique<PGBFS>(problem, option.get());

  throw std::runtime_error("No such search algorithm.");
}

}  // namespace pplanner
