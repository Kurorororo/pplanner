#ifndef LANDMARK_DETECTION_H_
#define LANDMARK_DETECTION_H_

#include <memory>

#include "sas_plus.h"
#include "heuristics/relaxed_sas_plus.h"
#include "landmark/landmark_graph.h"

namespace pplanner {

void IdentifyLandmarks(std::shared_ptr<const SASPlus> problem,
                       std::shared_ptr<const RelaxedSASPlus> r_problem,
                       std::shared_ptr<LandmarkGraph> graph);
} // namespace pplanner

#endif // LANDMARK_DETECTION_H_
