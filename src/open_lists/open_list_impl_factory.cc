#include "open_list/open_list_impl.h"

namespace pplanner {

std::unique_ptr<OpenListImpl> OpenListImplFactory(
    const std::string &tie_breaking) {
  return std::make_unique<FIFOOpenListImpl>();
}

} // namespace pplanner
