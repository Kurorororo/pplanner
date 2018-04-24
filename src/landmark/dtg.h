#ifndef DTG_H_
#define DTG_H_

#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "heuristic/graphplan.h"

namespace rwls {

class DTG {
 public:
  DTG() {}

  DTG(const DTG &dtg) {
    adjacent_lists_ = dtg.adjacent_lists_;
  }

  explicit DTG(const std::vector< std::unordered_set<int> > &adjacent_lists) {
    adjacent_lists_ = adjacent_lists;
  }

  ~DTG() {}

  inline DTG &operator=(const DTG &dtg) {
    adjacent_lists_ = dtg.adjacent_lists_;
    return *this;
  }

  void RemoveNode(int value);

  inline void SoftRemoveNode(int value) {
    deleted_.insert(value);
  }

  inline void RecoverSoftDelete() {
    deleted_.clear();
  }

  void RemoveNodesByRRPG(const Domain &domain, const PlanningGraph &rrpg,
                         int var, int goal_value);

  bool IsConnected(int start, int goal, int ignore=-1);

  void Print();

  static std::vector<DTG> InitializeDTGs(const Domain &domain);

 private:
  bool RecursiveIsConnected(int i, int goal);

  std::vector< std::unordered_set<int> > adjacent_lists_;
  std::unordered_set<int> closed_;
  std::unordered_set<int> deleted_;
};

} // namespace rwls

#endif // DTG_H_
