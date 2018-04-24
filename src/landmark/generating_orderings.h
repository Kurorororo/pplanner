#ifndef GENERATING_ORDERINGS_H_
#define GENERATING_ORDERINGS_H_

#include "domain/domain.h"
#include "heuristic/graphplan.h"
#include "landmark/landmark_graph.h"

namespace rwls {

void AddOrderings(const Domain &domain, const GraphSchema &schema,
                  LandmarkGraph *graph);

void HandleCycles(LandmarkGraph *graph);

}

#endif // GENERATING_ORDERINGS_H_
