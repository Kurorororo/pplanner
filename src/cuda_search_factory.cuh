#ifndef CUDA_SEARCH_FACTORY_H_
#define CUDA_SEARCH_FACTORY_H_

#include <memory>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "search.h"

namespace pplanner {

std::unique_ptr<Search> CudaSearchFactory(
    std::shared_ptr<const SASPlus> problem,
    const boost::property_tree::ptree &pt);

} // namespace pplanner

#endif // CUDA_SEARCH_FACTORY_H_
