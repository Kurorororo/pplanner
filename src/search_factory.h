#ifndef SEARCH_FACTORY_H_
#define SEARCH_FACTORY_H_

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "search.h"

namespace pplanner {

std::unique_ptr<Search> SearchFactory(std::shared_ptr<const SASPlus> problem,
                                      const boost::property_tree::ptree &pt,
                                      int max_expansion);



} // namespace pplanner

#endif // SEARCH_FACTORY_H_
