#include "cuda_search/cuda_bmrw.cuh"

#include "cuda_landmark/landmark_count_base.cuh"
#include "cuda_landmark/landmark_graph.cuh"

namespace pplanner {

__global__
void RandomWalk(bool *accepted, bool) {
}

void CudaBMRW::Init(const boost::property_tree::ptree &pt) {
  int closed_exponent = 22;

  if (auto closed_exponent_opt = pt.get_optional<int>("closed_exponent"))
    closed_exponent = closed_exponent_opt.get();

  graph_ = std::make_shared<SearchGraphWithLandmarks>(
      problem_, closed_exponent);

  lmcount_ = std::make_shared<LandmarkCount>(
      problem, graph, false, false, false, false);

  std::vector<std::shared_ptr<Evaluator> > evaluators{lmcount_};

  auto open_list_option = pt.get_child("open_list");
  open_list_ = OpenListFactory(open_list_option, evaluators);

  size_t ram = 5000000000;

  if (auto opt = pt.get_optional<size_t>("ram"))
    ram = opt.get();

  std::size_t size = graph_->ReserveByRAMSize(ram);
  sequences_.reserve(size);
}

} // namespace pplanner
