#include "sas_plus/mutex_groups.h"

namespace pplanner {

void MutexGroups::AddGroup(const std::vector<int> &group) {
  size_t size = groups_.size();
  groups_.resize(size + 1);

  for (auto f : group)
    groups_[size].insert(f);
}

bool MutexGroups::IsMutex(int f, int g) const {
  for (auto &group : groups_)
    if (group.find(f) != group.end() && group.find(g) != group.end())
      return true;

  return false;
}

} // pplanner
