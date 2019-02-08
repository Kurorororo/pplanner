#ifndef OPEN_LIST_FACTORY_H_
#define OPEN_LIST_FACTORY_H_

#include <memory>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "evaluator.h"
#include "open_list.h"
#include "open_lists/alternating_open_list.h"
#include "open_lists/preferred_open_list.h"
#include "open_lists/preferred_plus_open_list.h"
#include "open_lists/single_open_list.h"

namespace pplanner {

template<typename T = int>
std::unique_ptr<OpenList<T> > OpenListFactory(
    const boost::property_tree::ptree &pt,
    const std::vector<std::shared_ptr<Evaluator> > &evaluators) {
  std::string tie_breaking = "fifo";

  if (auto option = pt.get_optional<std::string>("tie_breaking"))
    tie_breaking = option.get();

  bool alternating = false;

  if (auto option = pt.get_optional<int>("alternating"))
    alternating = option.get() == 1;

  if (alternating)
    return std::make_unique<AlternatingOpenList<T> >(tie_breaking, evaluators);

  int preferred = 0;

  if (auto option = pt.get_optional<int>("preferred"))
    preferred = option.get();

  int n_boost = 0;

  if (auto option = pt.get_optional<int>("boost"))
    n_boost = option.get();

  switch (preferred) {
    case 0:
      return std::make_unique<SingleOpenList<T> >(tie_breaking, evaluators);
    case 1:
      return std::make_unique<PreferredOpenList<T> >(
          tie_breaking, evaluators, n_boost);
    case 2:
      return std::make_unique<PreferredPlusOpenList<T> >(
          tie_breaking, evaluators);
  }

  return std::make_unique<SingleOpenList<T> >(tie_breaking, evaluators);
}

template<typename T = int>
std::shared_ptr<OpenList<T> > SharedOpenListFactory(
    const boost::property_tree::ptree &pt,
    const std::vector<std::shared_ptr<Evaluator> > &evaluators) {
  std::string tie_breaking = "fifo";

  if (auto option = pt.get_optional<std::string>("tie_breaking"))
    tie_breaking = option.get();

  bool alternating = false;

  if (auto option = pt.get_optional<int>("alternating"))
    alternating = option.get() == 1;

  if (alternating)
    return std::make_shared<AlternatingOpenList<T> >(tie_breaking, evaluators);

  int preferred = 0;

  if (auto option = pt.get_optional<int>("preferred"))
    preferred = option.get();

  int n_boost = 0;

  if (auto option = pt.get_optional<int>("boost"))
    n_boost = option.get();

  switch (preferred) {
    case 0:
      return std::make_shared<SingleOpenList<T> >(tie_breaking, evaluators);
  }

  return std::make_shared<SingleOpenList<T> >(tie_breaking, evaluators);
}

} // namespace pplanner

#endif // OPEN_LIST_FACTORY_H_
