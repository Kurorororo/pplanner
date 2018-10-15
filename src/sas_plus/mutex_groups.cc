#include "sas_plus/mutex_groups.h"

#include <iostream>

namespace pplanner {

void MutexGroups::AddGroup(const std::vector<int> &group) {
  size_t size = groups_.size();
  groups_.resize(size + 1);

  for (auto f : group)
    groups_[size].insert(f);
}

bool MutexGroups::IsMutex(int f, int g) const {
  if (f == g) return false;

  for (auto &group : groups_)
    if (group.find(f) != group.end() && group.find(g) != group.end())
      return true;

  return false;
}

void MutexGroups::Dump() const {
  std::cout << groups_.size() << " mutex groups" << std::endl;

  for (auto &group : groups_) {
    for (auto f : group)
      std::cout << "fact" << f << ", ";

    std::cout << std::endl;
  }
}

} // pplanner
