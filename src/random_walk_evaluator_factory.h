#ifndef RANDOM_WALK_EVALUATOR_FACTORY_H_
#define RANDOM_WALK_EVALUATOR_FACTORY_H_

#include <memory>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include "random_walk_evaluator.h"
#include "sas_plus.h"

namespace pplanner {

std::shared_ptr<RandomWalkEvaluator> RandomWalkEvaluatorFactory(
    std::shared_ptr<const SASPlus> problem,
    const boost::property_tree::ptree &pt);


} // namespace pplanner

#endif // RANDOM_WALK_EVALUATOR_FACTORY_H_
