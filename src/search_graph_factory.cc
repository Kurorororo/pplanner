#include "search_graph_factory.h"

#include "search_graph/search_graph_with_timestamp.h"
#include "search_graph/search_graph_with_landmarks.h"

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

} // namespace pplanner
