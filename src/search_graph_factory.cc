#include "search_graph_factory.h"

#include "search_graph/search_graph_with_timestamp.h"
#include "search_graph/search_graph_with_landmarks.h"
#include "search_graph/distributed_search_graph_with_timestamp.h"
#include "search_graph/distributed_search_graph_with_landmarks.h"

namespace pplanner {

using std::make_shared;

std::shared_ptr<SearchGraph> SearchFactory(
    std::shared_ptr<const SASPlus> problem,
    int closed_exponent,
    bool use_landmark,
    bool dump_nodes) {
  if (use_landmark) {
    if (dump_nodes) {
      return  make_shared<SearchGraphWithTimestamp<SearchGraphWithLandmarks> >(
          problem, closed_exponent);
    }

    return  make_shared<SearchGraphWithLandmarks>(problem, closed_exponent);
  }

  if (dump_nodes) {
    return  make_shared<SearchGraphWithTimestamp<SearchGraph> >(
        problem, closed_exponent);
  }

  return  make_shared<SearchGraph>(problem, closed_exponent);
}

std::shared_ptr<DistributedSearchGraph> DistributedSearchGraphFactory(
    std::shared_ptr<const SASPlus> problem,
    int closed_exponent,
    int n_evaluators,
    int rank,
    bool use_landmark,
    bool dump_nodes) {
  if (use_landmark) {
    if (dump_nodes) {
      return  make_shared<DistributedSearchGraphWithTimestamp<
        DistributedSearchGraphWithLandmarks> >(
            problem, closed_exponent, n_evaluators, rank);
    }

    return  make_shared<DistributedSearchGraphWithLandmarks>(
        problem, closed_exponent, n_evaluators, rank);
  }

  if (dump_nodes) {
    return  make_shared<DistributedSearchGraphWithTimestamp<
      DistributedSearchGraph> >(problem, closed_exponent, n_evaluators, rank);
  }

  return  make_shared<DistributedSearchGraph>(
      problem, closed_exponent, n_evaluators, rank);
}

} // namespace pplanner
