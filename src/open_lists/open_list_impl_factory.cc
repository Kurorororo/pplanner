#include "open_lists/open_list_impl_factory.h"

#include "open_lists/fifo_open_list_impl.h"
#include "open_lists/lifo_open_list_impl.h"

namespace pplanner {

std::unique_ptr<OpenListImpl> OpenListImplFactory(
    const std::string &tie_breaking) {
  if (tie_breaking == "lifo")
    return std::unique_ptr<LIFOOpenListImpl>(new LIFOOpenListImpl());

  return std::unique_ptr<FIFOOpenListImpl>(new FIFOOpenListImpl());
}

} // namespace pplanner
