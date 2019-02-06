#ifndef OPEN_LIST_FACTORY_H_
#define OPEN_LIST_FACTORY_H_

#include <memory>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "open_list.h"

namespace pplanner {

std::unique_ptr<OpenList> OpenListFactory(
    const boost::property_tree::ptree &pt,
    const std::vector<std::shared_ptr<Evaluator> > &evaluators);

std::shared_ptr<OpenList> SharedOpenListFactory(
    const boost::property_tree::ptree &pt,
    const std::vector<std::shared_ptr<Evaluator> > &evaluators);

} // namespace pplanner

#endif // OPEN_LIST_FACTORY_H_
