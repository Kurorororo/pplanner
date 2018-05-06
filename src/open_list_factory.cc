#include "open_list.h"

#include "open_lists/preferred_open_list.h"
#include "open_lists/single_open_list.h"

namespace pplanner {

std::unique_ptr<OpenList> OpenListFactory(
    const std::string &tie_breaking,
    const std::vector<std::shared_ptr<Evaluator> > &evaluators,
    bool preferred) {
  if (preferred)
    return std::unique_ptr<PreferredOpenList>(
        new PreferredOpenList(tie_breaking, evaluators));

  return std::unique_ptr<SingleOpenList>(
      new SingleOpenList(tie_breaking, evaluators));
}

} // namespace pplanner

