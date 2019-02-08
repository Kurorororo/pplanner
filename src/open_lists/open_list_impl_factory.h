#ifndef OPEN_LIST_IMPL_FACTORY_H_
#define OPEN_LIST_IMPL_FACTORY_H_

#include <memory>
#include <string>

#include "open_lists/open_list_impl.h"
#include "open_lists/fifo_open_list_impl.h"
#include "open_lists/lifo_open_list_impl.h"
#include "open_lists/ro_open_list_impl.h"

namespace pplanner {

template<typename T>
std::shared_ptr<OpenListImpl<T> > OpenListImplFactory(
    const std::string &tie_breaking) {
  if (tie_breaking == "lifo")
    return std::make_shared<LIFOOpenListImpl<T> >();

  if (tie_breaking == "ro")
    return std::make_shared<ROOpenListImpl<T> >();

  return std::make_shared<FIFOOpenListImpl<T> >();
}

} // namespace pplanner

#endif // OPEN_LIST_IMPL_FACTORY_H_
