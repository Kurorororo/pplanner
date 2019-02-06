#include "cuda_search_factory.cuh"

#include "cuda_search/cuda_bmrw.cuh"
#include "cuda_search/cuda_bmrw_gbfs.cuh"
#include "cuda_search/cuda_bmrw_gbfs_share.cuh"
#include "cuda_search/cuda_bmrw_gbfs_focus.cuh"
#include "cuda_search/cuda_bmrw_gbfs_focus2.cuh"
#include "cuda_search/cuda_hetro_gbfs.cuh"
#include "cuda_search/cuda_hdgbfs.cuh"
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

  if (search.get() == "bmrw_gbfs")
    return std::unique_ptr<CudaBMRWGBFS>(
        new CudaBMRWGBFS(problem, option.get()));

  if (search.get() == "bmrw_gbfs_share")
    return std::unique_ptr<CudaBMRWGBFSShare>(
        new CudaBMRWGBFSShare(problem, option.get()));

  if (search.get() == "bmrw_gbfs_focus")
    return std::unique_ptr<CudaBMRWGBFSFocus>(
        new CudaBMRWGBFSFocus(problem, option.get()));

  if (search.get() == "bmrw_gbfs_focus2")
    return std::unique_ptr<CudaBMRWGBFSFocus2>(
        new CudaBMRWGBFSFocus2(problem, option.get()));

  if (search.get() == "hetro_gbfs")
    return std::unique_ptr<CudaHetroGBFS>(
        new CudaHetroGBFS(problem, option.get()));

  if (search.get() == "hdgbfs")
    return std::unique_ptr<CudaHDGBFS>(new CudaHDGBFS(problem, option.get()));

  if (search.get() == "rwagbfs")
    return std::unique_ptr<CudaRWAGBFS>(new CudaRWAGBFS(problem, option.get()));

  throw std::runtime_error("No such search algorithm.");
}

} // namespace pplanner
