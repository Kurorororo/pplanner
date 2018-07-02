#ifndef KGBFS_H_
#define KGBFS_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "search/gbfs.h"

namespace pplanner {

class KGBFS : public GBFS {
 public:
  KGBFS(std::shared_ptr<const SASPlus> problem,
        const boost::property_tree::ptree &pt) : GBFS(problem, pt), k_(2) {
    if (auto k = pt.get_optional<int>("k"))
      k_ = k.get();
  }

  ~KGBFS() {}

  int Search() override;

 private:
  int k_;
};

} // namespace pplanner

#endif // KGBFS_H_
