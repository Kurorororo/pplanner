#include "search_graph_factory.h"

#include "search_graph/search_graph_with_costs.h"
#include "search_graph/search_graph_with_landmarks.h"
#include "search_graph/search_graph_with_timestamp.h"

namespace pplanner {

using std::make_shared;

std::shared_ptr<SearchGraph> SearchGraphFactory(
    std::shared_ptr<const SASPlus> problem,
    int closed_exponent,
    bool keep_cost,
    bool use_landmark,
    bool dump_nodes) {
  if (keep_cost) {
    if (use_landmark) {
      if (dump_nodes) {
        return  make_shared<SearchGraphWithTimestamp<SearchGraphWithCosts<
          SearchGraphWithLandmarks> > >(problem, closed_exponent);
      }

      return  make_shared<SearchGraphWithCosts<SearchGraphWithLandmarks> >(
          problem, closed_exponent);
    }

    if (dump_nodes) {
      return  make_shared<SearchGraphWithTimestamp<SearchGraphWithCosts<
        SearchGraph> > >(problem, closed_exponent);
    }

    return make_shared<SearchGraphWithCosts<SearchGraph> >(
        problem, closed_exponent);
  }

  if (use_landmark) {
    if (dump_nodes) {
      return make_shared<SearchGraphWithTimestamp<SearchGraphWithLandmarks> >(
          problem, closed_exponent);
    }

    return  make_shared<SearchGraphWithLandmarks>(problem, closed_exponent);
  }

  if (dump_nodes) {
    return  make_shared<SearchGraphWithTimestamp<SearchGraph> >(
        problem, closed_exponent);
  }

  return make_shared<SearchGraph>(problem, closed_exponent);
}

} // namespace pplanner
