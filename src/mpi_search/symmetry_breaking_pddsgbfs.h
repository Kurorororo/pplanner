#ifndef SBPDDSGBFS_H_
#define SBPDDSGBFS_H_

#include <memory>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "mpi_search/symmetry_breaking_hdgbfs.h"

namespace pplanner {

class SBPDDSGBFS : public SBHDGBFS {
 public:
  SBPDDSGBFS(std::shared_ptr<const SASPlus> problem,
           const boost::property_tree::ptree &pt)
    : SBHDGBFS(problem, pt),
      steal_best_(false),
      steal_better_(false) {
    if (auto opt = pt.get_optional<int>("steal_best"))
      steal_best_ = true;

    if (auto opt = pt.get_optional<int>("steal_better"))
      steal_better_ = true;
  }

  ~SBPDDSGBFS() {
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

  bool steal_best_;
  bool steal_better_;
};

} // namespace pplanner

#endif // SBPDDSGBFS_H_