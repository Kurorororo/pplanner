#include "cuda_search_factory.cuh"

#include "cuda_search/cuda_bmrw.cuh"
#include "cuda_search/cuda_rwagbfs.cuh"

namespace pplanner {

std::unique_ptr<Search> CudaSearchFactory(
    std::shared_ptr<const SASPlus> problem,
    const boost::property_tree::ptree &pt) {
  auto search = pt.get_optional<std::string>("search");
  if (!search) throw std::runtime_error("Parameter search is needed.");

  auto option = pt.get_child_optional("option");
  if (!option) throw std::runtime_error("Parameter option is needed.");

  if (search.get() == "bmrw")
    return std::unique_ptr<CudaBMRW>(new CudaBMRW(problem, option.get()));

  if (search.get() == "rwagbfs")
    return std::unique_ptr<CudaRWAGBFS>(new CudaRWAGBFS(problem, option.get()));

  throw std::runtime_error("No such search algorithm.");
}

} // namespace pplanner
