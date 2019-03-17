#include "evaluator_factory.h"

#include "heuristics/additive.h"
#include "heuristics/blind.h"
#include "heuristics/ff.h"
#include "heuristics/ff_add.h"
#include "heuristics/hmax.h"
#include "heuristics/landmark_count.h"
#include "heuristics/new_operator.h"
#include "heuristics/weighted_evaluator.h"
#include "heuristics/width.h"
#include "heuristics/zobrist_ip_tiebreaking.h"
#include "search_graph/search_graph_with_landmarks.h"

namespace pplanner {

std::shared_ptr<Evaluator> EvaluatorFactory(
    std::shared_ptr<const SASPlus> problem, std::shared_ptr<SearchGraph> graph,
    std::shared_ptr<Evaluator> friend_evaluator,
    const boost::property_tree::ptree &pt) {
  auto name = pt.get_optional<std::string>("name");

  if (!name) throw std::runtime_error("Heuristic name is needed.");

  if (name.get() == "blind") return std::make_shared<Blind>(problem);

  if (name.get() == "add") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    return std::make_shared<Additive>(problem, simplify, unit_cost);
  }

  if (name.get() == "hmax") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    return std::make_shared<Hmax>(problem, simplify, unit_cost);
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

    std::string tie_break = "cpp";
    auto t_option = pt.get_optional<std::string>("option.tie_break");
    if (t_option) tie_break = t_option.get();

    return std::make_shared<FFAdd>(problem, simplify, unit_cost, tie_break,
                                   more_helpful);
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

    return std::make_shared<LandmarkCount>(problem, graph, unit_cost, simplify,
                                           use_rpg_table, more_helpful);
  }

  if (name.get() == "zobrist_ip_tiebreaking")
    return std::make_shared<ZobristIPTiebreaking>(problem);

  if (name.get() == "weighted") {
    auto child = pt.get_child_optional("heuristic");
    if (!child) throw std::runtime_error("weighted need heuristic.");

    int weight = 1;
    if (auto option = pt.get_optional<int>("weight")) weight = option.get();

    return std::make_shared<WeightedEvaluator>(weight, problem, graph,
                                               friend_evaluator, child.get());
  }

  if (name.get() == "weighted_cache") {
    auto e = std::dynamic_pointer_cast<WeightedEvaluator>(friend_evaluator);

    if (e == nullptr) throw std::runtime_error("weighted_cache need weighted.");

    return std::make_shared<WeightedHeuristicCache>(e);
  }

  throw std::runtime_error("No such heuristic.");
}

}  // namespace pplanner
