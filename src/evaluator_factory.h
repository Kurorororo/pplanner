#ifndef EVALUATOR_FACTORY_H_
#define EVALUATOR_FACTORY_H_

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "sas_plus.h"
#include "search_graph.h"

namespace pplanner {

std::shared_ptr<Evaluator> EvaluatorFactory(
    std::shared_ptr<const SASPlus> problem,
    std::shared_ptr<SearchGraph> graph,
    std::shared_ptr<Evaluator> friend_evaluator,
    const boost::property_tree::ptree &pt);


} // namespace pplanner

#endif // EVALUATOR_FACTORY_H_
