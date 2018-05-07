#ifndef ACTION_ELIMINATION_H_
#define ACTION_ELIMINATION_H_

#include <vector>

#include "sas_plus.h"

namespace pplanner {

std::vector<int> ActionElimination(const SASPlus &problem,
                                   std::vector<int> plan);

} // namespace pplanner

#endif // ACTION_ELIMINATION_H_
