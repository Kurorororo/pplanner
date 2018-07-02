#ifndef MPI_SEARCH_FACTORY_H_
#define MPI_SEARCH_FACTORY_H_

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "search.h"

namespace pplanner {

std::unique_ptr<Search> MpiSearchFactory(
    std::shared_ptr<const SASPlus> &problem,
    const boost::property_tree::ptree &pt);

} // namespace pplanner

#endif // MPI_SEARCH_FACTORY_H_
