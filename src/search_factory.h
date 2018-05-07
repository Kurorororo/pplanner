#ifndef SEARCH_FACTORY_H_
#define SEARCH_FACTORY_H_

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "search.h"

namespace pplanner {

std::unique_ptr<Search> SearchFactory(const std::shared_ptr<SASPlus> &problem,
                                      const boost::property_tree::ptree &pt);



} // namespace pplanner

#endif // SEARCH_FACTORY_H_
