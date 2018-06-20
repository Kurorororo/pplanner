#include "random_walk_evaluator_factory.h"

#include "heuristics/additive.h"
#include "heuristics/blind.h"
#include "heuristics/ff.h"
#include "heuristics/ff_add.h"
#include "heuristics/landmark_count.h"
#include "heuristics/new_operator.h"
#include "heuristics/width.h"

namespace pplanner {

std::shared_ptr<RandomWalkEvaluator> RandomWalkEvaluatorFactory(
    std::shared_ptr<const SASPlus> problem,
    const boost::property_tree::ptree &pt) {

  auto name = pt.get_optional<std::string>("name");

  if (!name) throw std::runtime_error("Heuristic name is needed.");

  if (name.get() == "blind")
    return std::make_shared<RWBlind>(problem);

  if (name.get() == "add") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    return std::make_shared<RWAdditive>(problem, simplify, unit_cost);
  }

  if (name.get() == "fa") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    bool more_helpful = false;

    option = pt.get_optional<int>("option.more");
    if (option) more_helpful = option.get() == 1;

    return std::make_shared<RWFFAdd>(
        problem, simplify, unit_cost, more_helpful);
  }

  if (name.get() == "ff") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    return std::make_shared<RWFF>(
        problem, simplify, unit_cost);
  }

  if (name.get() == "width") {
    bool is_ge_1 = false;

    auto option = pt.get_optional<int>("option.ge_1");
    if (option) is_ge_1 = option.get() == 1;

    return std::make_shared<RWWidth>(problem, is_ge_1);
  }

  if (name.get() == "new_op") {
    return std::make_shared<RWNewOperator>(problem);
  }

  if (name.get() == "lmc") {
    bool unit_cost = problem->metric() == 0;
    if (auto option = pt.get_optional<int>("option.unit_cost"))
      unit_cost = option.get() == 1;

    bool simplify = false;
    if (auto option = pt.get_optional<int>("option.simplify"))
      simplify = option.get() == 1;

    bool use_rpg_table = false;
    if (auto option = pt.get_optional<int>("option.rpg_table"))
      use_rpg_table = option.get() == 1;

    bool more_helpful = false;
    if (auto option = pt.get_optional<int>("option.more"))
      more_helpful = option.get() == 1;

    return std::make_shared<RWLandmarkCount>(
        problem, unit_cost, simplify, use_rpg_table, more_helpful);
  }

  throw std::runtime_error("No such heuristic.");
}

} // namespace pplanner
