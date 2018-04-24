#ifndef LANDMARK_DETECTION_H_
#define LANDMARK_DETECTION_H_

#include "domain/domain.h"
#include "domain/state.h"
#include "heuristic/graphplan.h"
#include "landmark/landmark_graph.h"

namespace rwls {

void IdentifyLandmarks(const Domain &domain, const GraphSchema &schema,
                       LandmarkGraph *graph);

} // namespace rwls

#endif // LANDMARK_DETECTION_H_
