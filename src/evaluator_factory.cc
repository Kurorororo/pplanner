#include "evaluator_factory.h"

#include "heuristics/additive.h"
#include "heuristics/blind.h"
#include "heuristics/ff.h"
#include "heuristics/ff_add.h"
#include "heuristics/new_operator.h"
#include "heuristics/width.h"

namespace pplanner {

std::shared_ptr<Evaluator> EvaluatorFactory(
    std::shared_ptr<const SASPlus> problem,
    const boost::property_tree::ptree &pt) {

  auto name = pt.get_optional<std::string>("name");

  if (!name) throw std::runtime_error("Heuristic name is needed.");

  if (name.get() == "blind")
    return std::make_shared<Blind>(problem);

  if (name.get() == "add") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    return std::make_shared<Additive>(problem, simplify, unit_cost);
  }

  if (name.get() == "fa") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    return std::make_shared<FFAdd>(problem, simplify, unit_cost);
  }

  if (name.get() == "ff") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    return std::make_shared<FF>(problem, simplify, unit_cost);
  }

  if (name.get() == "width") {
    bool is_ge_1 = false;

    auto option = pt.get_optional<int>("option.ge_1");
    if (option) is_ge_1 = option.get() == 1;

    return std::make_shared<Width>(problem, is_ge_1);
  }

  if (name.get() == "new_op") {
    return std::make_shared<NewOperator>(problem);
  }

  throw std::runtime_error("No such heuristic.");
}

} // namespace pplanner
