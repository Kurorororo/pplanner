#include "open_list_factory.h"

#include "open_lists/preferred_open_list.h"
#include "open_lists/preferred_plus_open_list.h"
#include "open_lists/single_open_list.h"

namespace pplanner {

std::unique_ptr<OpenList> OpenListFactory(
    const boost::property_tree::ptree &pt,
    const std::vector<std::shared_ptr<Evaluator> > &evaluators) {
  std::string tie_breaking = "fifo";

  if (auto option = pt.get_optional<std::string>("tie_breaking"))
    tie_breaking = option.get();

  int preferred = 0;

  if (auto option = pt.get_optional<int>("preferred"))
    preferred = option.get();

  int n_boost = 0;

  if (auto option = pt.get_optional<int>("boost"))
    n_boost = option.get();

  switch (preferred) {
    case 0:
      return std::unique_ptr<SingleOpenList>(
        new SingleOpenList(tie_breaking, evaluators));
    case 1:
      return std::unique_ptr<PreferredOpenList>(
        new PreferredOpenList(tie_breaking, evaluators, n_boost));
    case 2:
      return std::unique_ptr<PreferredPlusOpenList>(
        new PreferredPlusOpenList(tie_breaking, evaluators));
  }

  return std::unique_ptr<SingleOpenList>(
    new SingleOpenList(tie_breaking, evaluators));
}

std::shared_ptr<OpenList> SharedOpenListFactory(
    const boost::property_tree::ptree &pt,
    const std::vector<std::shared_ptr<Evaluator> > &evaluators) {
  std::string tie_breaking = "fifo";

  if (auto option = pt.get_optional<std::string>("tie_breaking"))
    tie_breaking = option.get();

  int preferred = 0;

  if (auto option = pt.get_optional<int>("preferred"))
    preferred = option.get();

  int n_boost = 0;

  if (auto option = pt.get_optional<int>("boost"))
    n_boost = option.get();

  switch (preferred) {
    case 0:
      return std::make_shared<SingleOpenList>(tie_breaking, evaluators);
    case 1:
      return std::make_shared<PreferredOpenList>(
          tie_breaking, evaluators, n_boost);
    case 2:
      return std::make_shared<PreferredPlusOpenList>(tie_breaking, evaluators);
  }

  return std::make_shared<SingleOpenList>(tie_breaking, evaluators);
}

} // namespace pplanner

