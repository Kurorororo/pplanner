#ifndef MUTEX_GROUPS_H_
#define MUTEX_GROUPS_H_

#include <unordered_set>
#include <vector>

namespace pplanner {

class MutexGroups {
 public:
  MutexGroups() {}

  explicit MutexGroups(size_t size) { groups_.resize(size); }

  size_t size() const { return groups_.size(); };

  void AddGroup() { groups_.resize(groups_.size() + 1); }

  void Insert(int i, int fact) { groups_[i].insert(fact); }

  bool IsMutex(int f, int g) const;

 private:
  std::vector<std::unordered_set<int> > groups_;
};

} // namespace pplanne

#endif // MUTEX_GROUPS_H_
