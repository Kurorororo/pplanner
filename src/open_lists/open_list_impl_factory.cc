#include "open_lists/open_list_impl_factory.h"

#include "open_lists/fifo_open_list_impl.h"
#include "open_lists/lifo_open_list_impl.h"
#include "open_lists/ro_open_list_impl.h"

namespace pplanner {

std::shared_ptr<OpenListImpl> OpenListImplFactory(
    const std::string &tie_breaking) {
  if (tie_breaking == "lifo")
    return std::make_shared<LIFOOpenListImpl>();

  if (tie_breaking == "ro")
    return std::make_shared<ROOpenListImpl>();

  return std::make_shared<FIFOOpenListImpl>();
}

} // namespace pplanner
