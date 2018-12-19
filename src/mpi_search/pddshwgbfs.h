#ifndef PDDSHWGBFS_H_
#define PDDSHWGBFS_H_

#include <memory>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "mpi_search/hdgbfs.h"

namespace pplanner {

class PDDSHWGBFS : public HDGBFS {
 public:
  PDDSHWGBFS(std::shared_ptr<const SASPlus> problem,
           const boost::property_tree::ptree &pt)
    : HDGBFS(problem, pt),
      steal_better_(false) {
    if (auto opt = pt.get_optional<int>("steal_better"))
      steal_better_ = true;
  }

  ~PDDSHWGBFS() {
    Flush(kRegainTag);
  }

  int Search() override;

  void CallbackOnReceiveNode(int source, const unsigned char *d, bool no_node)
    override;

  void CallbackOnReceiveAllNodes() override { SendNodes(kRegainTag); }

  void RegainNodes();

  static constexpr int kRegainTag = 4;

 private:
  void FlushRegainTag();

  bool steal_better_;
  std::vector<bool> hws_;
};

} // namespace pplanner

#endif // PDDSHWGBFS_H_