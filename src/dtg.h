#ifndef DTG_H_
#define DTG_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

class DTG {
 public:
  DTG() {}

  explicit DTG(const DTG &dtg) {
    adjacent_lists_ = dtg.adjacent_lists_;
  }

  explicit DTG(const std::vector<std::vector<int> > &adjacent_lists) {
    adjacent_lists_ = adjacent_lists;
  }

  ~DTG() {}

  DTG &operator=(const DTG &dtg) {
    adjacent_lists_ = dtg.adjacent_lists_;

    return *this;
  }

  void RemoveNode(int value);

  void SoftRemoveNode(int value) { deleted_.insert(value); }

  void RecoverSoftDelete() { deleted_.clear(); }

  bool IsConnected(int start, int goal, int ignore=-1);

  void Dump() const;

 private:
  bool RecursiveIsConnected(int i, int goal);

  std::vector<std::vector<int> > adjacent_lists_;
  std::unordered_set<int> closed_;
  std::unordered_set<int> deleted_;
};

std::vector<DTG> InitializeDTGs(std::shared_ptr<const SASPlus> problem);

} // namespace pplanner

#endif // DTG_H_
