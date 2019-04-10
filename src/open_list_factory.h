#ifndef OPEN_LIST_FACTORY_H_
#define OPEN_LIST_FACTORY_H_

#include <memory>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "open_list.h"
#include "open_lists/preferred_open_list.h"
#include "open_lists/preferred_plus_open_list.h"
#include "open_lists/single_open_list.h"

namespace pplanner {

template <typename T = std::vector<int>, typename U = int>
std::unique_ptr<OpenList<T, U> > OpenListFactory(
    const boost::property_tree::ptree &pt) {
  std::string tie_breaking = "fifo";

  if (auto option = pt.get_optional<std::string>("tie_breaking"))
    tie_breaking = option.get();

  int preferred = 0;

  if (auto option = pt.get_optional<int>("preferred")) preferred = option.get();

  int n_boost = 0;

  if (auto option = pt.get_optional<int>("boost")) n_boost = option.get();

  switch (preferred) {
    case 0:
      return std::make_unique<SingleOpenList<T, U> >(tie_breaking);
    case 1:
      return std::make_unique<PreferredOpenList<T, U> >(tie_breaking, n_boost);
    case 2:
      return std::make_unique<PreferredPlusOpenList<T, U> >(tie_breaking);
  }

  return std::make_unique<SingleOpenList<T, U> >(tie_breaking);
}

template <typename T = std::vector<int>, typename U = int>
std::shared_ptr<OpenList<T, U> > SharedOpenListFactory(
    const boost::property_tree::ptree &pt) {
  std::string tie_breaking = "fifo";

  if (auto option = pt.get_optional<std::string>("tie_breaking"))
    tie_breaking = option.get();

  int preferred = 0;

  if (auto option = pt.get_optional<int>("preferred")) preferred = option.get();

  int n_boost = 0;

  if (auto option = pt.get_optional<int>("boost")) n_boost = option.get();

  switch (preferred) {
    case 0:
      return std::make_shared<SingleOpenList<T, U> >(tie_breaking);
    case 1:
      return std::make_shared<PreferredOpenList<T, U> >(tie_breaking, n_boost);
    case 2:
      return std::make_shared<PreferredPlusOpenList<T, U> >(tie_breaking);
  }

  return std::make_shared<SingleOpenList<T, U> >(tie_breaking);
}
}  // namespace pplanner

#endif  // OPEN_LIST_FACTORY_H_
