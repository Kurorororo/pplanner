#ifndef OPEN_LIST_FACTORY_H_
#define OPEN_LIST_FACTORY_H_

#include <memory>
#include <string>

#include "open_list.h"

namespace pplanner {

std::unique_ptr<OpenList> OpenListFactory(
    const std::string &tie_breaking,
    const std::vector<std::shared_ptr<Evaluator> > &evaluators,
    bool preferred,
    int n_boost);

} // namespace pplanner

#endif // OPEN_LIST_FACTORY_H_
