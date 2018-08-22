#ifndef SEARCH_GRAPH_FACTORY_H_
#define SEARCH_GRAPH_FACTORY_H_

#include <memory>

#include "search_graph.h"

namespace pplanner {

std::shared_ptr<SearchGraph> SearchGraphFactory(
    std::shared_ptr<const SASPlus> problem,
    int closed_exponent,
    bool keep_cost,
    bool use_landmark,
    bool dump_nodes);

} // namespace pplanner

#endif // SEARCH_GRAPH_FACTORY_H_
