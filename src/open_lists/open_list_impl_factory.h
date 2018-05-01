#ifndef OPEN_LIST_IMPL_FACTORY_H_
#define OPEN_LIST_IMPL_FACTORY_H_

#include <memory>
#include <string>

#include "open_list/open_list_impl.h"
#include "open_list/fifo_open_list_impl.h"

namespace pplanner {

std::unique_ptr<OpenListImpl> OpenListImplFactory();

} // namespace pplanner

#endif // OPEN_LIST_IMPL_FACTORY_H_
