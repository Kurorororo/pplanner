#include "open_lists/open_list_impl_factory.h"

namespace pplanner {

std::unique_ptr<OpenListImpl> OpenListImplFactory(
    const std::string &tie_breaking) {
  return std::unique_ptr<FIFOOpenListImpl>(new FIFOOpenListImpl());
}

} // namespace pplanner
