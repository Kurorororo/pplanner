#ifndef CUDA_BMRW_H_
#define CUDA_BMRW_H_

#include <memory>
#include <vector>

#include "sas_plus.h"
#include "search.h"
#include "successor_generator.h"
#include "search_graph.h"

namespace pplanner {

class CudaBMRW : public Search {
 public:
  CudaBMRW(std::shared_ptr<const SASPlus> problem,
           const boost::property_tree::ptree &pt)
    : generated_(0),
      expanded_(0),
      evaluated_(0),
      dead_ends_(0),
      problem_(problem),
      generator_(std::make_shared<SuccessorGenerator>(problem)),
      graph_(nullptr),
      lmcount_(nullptr),
      open_list_(nullptr) { Init(pt); }

  ~CudaBMRW() {}

  std::vector<int> Plan() override;

  void DumpStatistics() const override;

  int Search();

 private:
  void Init(const boost::property_tree::ptree &pt);

  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int best_h_;
  std::vector<std::vector<int> > sequences_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraphWithLandmarks> graph_;
  std::shared_ptr<LandmarkCount> lmcount_;
  std::unique_ptr<OpenList> open_list_;
}

} // namespace pplaner

#endif // CUDA_BMRW_H_
