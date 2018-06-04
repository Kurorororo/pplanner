#ifndef GENERATING_ORDERINGS_H_
#define GENERATING_ORDERINGS_H_

#include "sas_plus.h"
#include "heuristics/relaxed_sas_plus.h"
#include "landmark/landmark_graph.h"

namespace pplanner {

void AddOrderings(std::shared_ptr<const SASPlus> problem,
                  std::shared_ptr<const RelaxedSASPlus> r_problem,
                  std::shared_ptr<LandmarkGraph> graph);

void HandleCycles(std::shared_ptr<LandmarkGraph> graph);

} // pplanne

#endif // GENERATING_ORDERINGS_H_
