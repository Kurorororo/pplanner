#ifndef MUTEX_GROUPS_H_
#define MUTEX_GROUPS_H_

#include <unordered_set>
#include <vector>

namespace pplanner {

class MutexGroups {
 public:
  void Reserve(size_t size) { groups_.reserve(size); }

  size_t size() const { return groups_.size(); };

  void AddGroup(const std::vector<int> &group);

  bool IsMutex(int f, int g) const;

  void Dump() const;

 private:
  std::vector<std::unordered_set<int> > groups_;
};

} // namespace pplanne

#endif // MUTEX_GROUPS_H_
