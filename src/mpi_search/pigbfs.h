#ifndef PIGBFS_H_
#define PIGBFS_H_

#include <memory>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "sas_plus.h"
#include "mpi_search/hdgbfs.h"

namespace pplanner {

class PIGBFS : public HDGBFS {
 public:
  PIGBFS(std::shared_ptr<const SASPlus> problem,
         const boost::property_tree::ptree &pt)
    : HDGBFS(problem, pt) {}

  ~PIGBFS() {}

  int Search() override;
};

} // namespace pplanner

#endif // PIGBFS_H_
