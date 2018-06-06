#include "evaluator_factory.h"

#include "heuristics/additive.h"
#include "heuristics/blind.h"
#include "heuristics/ff.h"
#include "heuristics/ff_add.h"
#include "heuristics/landmark_count.h"
#include "heuristics/new_operator.h"
#include "heuristics/width.h"
#include "search_graph/search_graph_with_landmarks.h"

namespace pplanner {

std::shared_ptr<Evaluator> EvaluatorFactory(
    std::shared_ptr<const SASPlus> problem,
    std::shared_ptr<SearchGraph> graph,
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

    bool more_helpful = false;

    option = pt.get_optional<int>("option.more");
    if (option) more_helpful = option.get() == 1;

    return std::make_shared<FFAdd>(problem, simplify, unit_cost, more_helpful);
  }

  if (name.get() == "ff") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    bool common_precond = false;

    option = pt.get_optional<int>("option.common_precond");
    if (option) common_precond = option.get() == 1;

    return std::make_shared<FF>(problem, simplify, unit_cost, common_precond);
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

  if (name.get() == "lmc") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    if (auto g = std::dynamic_pointer_cast<SearchGraphWithLandmarks>(graph)) {
      return std::make_shared<LandmarkCount>(problem, g, simplify);
    } else {
      throw std::runtime_error("Use SearchGraphWithLandmarks for lmcount.");
    }
  }

  throw std::runtime_error("No such heuristic.");
}

} // namespace pplanner
