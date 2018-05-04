#include "evaluator_factory.h"

#include "heuristics/additive.h"
#include "heuristics/blind.h"
#include "heuristics/ff.h"
#include "heuristics/ff_add.h"

namespace pplanner {

std::unique_ptr<Evaluator> EvaluatorFactory(
    std::shared_ptr<const SASPlus> problem,
    const boost::property_tree::ptree &pt) {

  auto name = pt.get_optional<std::string>("name");

  if (!name) throw std::runtime_error("Heuristic name is needed.");

  if (name.get() == "blind")
    return std::unique_ptr<Evaluator>(new Blind(problem));

  if (name.get() == "add") {
    bool simplify = false;

    auto option = pt.get_optional<int>("options.simplify");
    if (option) simplify = option.get() == 1;

    return std::unique_ptr<Evaluator>(new Additive(problem, simplify));
  }

  if (name.get() == "fa") {
    bool simplify = false;

    auto option = pt.get_optional<int>("options.simplify");
    if (option) simplify = option.get() == 1;

    return std::unique_ptr<Evaluator>(new FFAdd(problem, simplify));
  }

  if (name.get() == "ff") {
    bool simplify = false;

    auto option = pt.get_optional<int>("options.simplify");
    if (option) simplify = option.get() == 1;

    return std::unique_ptr<Evaluator>(new FF(problem, simplify));
  }

  throw std::runtime_error("No such heuristic.");
}

} // namespace pplanner
