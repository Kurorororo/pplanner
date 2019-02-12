#ifndef HEURISTIC_FACTORY_H_
#define HEURISTIC_FACTORY_H_

#include <memory>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "multithread_search/ff.h"
#include "multithread_search/ff_add.h"
#include "multithread_search/heuristic.h"
#include "multithread_search/landmark_count.h"


namespace pplanner {

template<typename T>
std::shared_ptr<Heuristic<T> > HeuristicFactory(
    std::shared_ptr<const SASPlus> problem,
    const boost::property_tree::ptree &pt) {

  auto name = pt.get_optional<std::string>("name");

  if (!name) throw std::runtime_error("Heuristic name is needed.");

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

    return std::make_shared<MFFAdd<T> >(problem, simplify, unit_cost,
                                        more_helpful);
  }

  if (name.get() == "ff") {
    bool simplify = false;

    auto option = pt.get_optional<int>("option.simplify");
    if (option) simplify = option.get() == 1;

    bool unit_cost = problem->metric() == 0;

    option = pt.get_optional<int>("option.unit_cost");
    if (option && !unit_cost) unit_cost = option.get() == 1;

    return std::make_shared<MFF<T> >(problem, simplify, unit_cost);
  }

  if (name.get() == "lmc") {
    bool unit_cost = problem->metric() == 0;
    if (auto option = pt.get_optional<int>("option.unit_cost"))
      unit_cost = option.get() == 1;

    bool use_rpg_table = false;
    if (auto option = pt.get_optional<int>("option.rpg_table"))
      use_rpg_table = option.get() == 1;

    bool more_helpful = false;
    if (auto option = pt.get_optional<int>("option.more"))
      more_helpful = option.get() == 1;

    return std::make_shared<MLandmarkCount<T> >(
        problem, unit_cost, use_rpg_table, more_helpful);
  }

  throw std::runtime_error("No such heuristic.");
}

} // namespace pplanner

#endif // HEURISTIC_FACTORY_H_