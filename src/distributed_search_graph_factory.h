#ifndef DISTRIBUTED_SEARCH_GRAPH_FACTORY_H_
#define DISTRIBUTED_SEARCH_GRAPH_FACTORY_H_

#include <memory>

#include "search_graph/distributed_search_graph.h"

namespace pplanner {

std::shared_ptr<DistributedSearchGraph> DistributedSearchGraphFactory(
    std::shared_ptr<const SASPlus> problem,
    int closed_exponent,
    int n_evaluators,
    int rank,
    bool use_landmark,
    bool dump_nodes);

} // namespace pplanner

#endif // DISTRIBUTED_SEARCH_GRAPH_FACTORY_H_
