#include "problem/mutex_groups.h"

namespace pplanner {

bool MutexGroups::IsMutex(int f, int g) const {
  for (auto &group : groups_)
    if (group.find(f) != group.end() && group.find(g) != group.end())
      return true;

  return false;
}

} // pplanner
