#ifndef OPEN_LIST_IMPL_FACTORY_H_
#define OPEN_LIST_IMPL_FACTORY_H_

#include <memory>
#include <string>

#include "open_lists/open_list_impl.h"
#include "open_lists/fifo_open_list_impl.h"

namespace pplanner {

std::shared_ptr<OpenListImpl> OpenListImplFactory(
    const std::string &tie_breaking);

} // namespace pplanner

#endif // OPEN_LIST_IMPL_FACTORY_H_
