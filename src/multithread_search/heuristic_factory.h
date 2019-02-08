#ifndef HEURISTIC_FACTORY_H_
#define HEURISTIC_FACTORY_H_

#include <memory>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "multithread_search/heuristic.h"

namespace pplanner {

std::shared_ptr<Heuristic> HeuristicFactory(
    std::shared_ptr<const SASPlus> problem,
    const boost::property_tree::ptree &pt);


} // namespace pplanner

#endif // HEURISTIC_FACTORY_H_
