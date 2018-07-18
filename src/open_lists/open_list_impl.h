#ifndef OPEN_LIST_IMPL_H_
#define OPEN_LIST_IMPL_H_

#include <cstddef>

#include <vector>

namespace pplanner {

class OpenListImpl {
 public:
  virtual ~OpenListImpl() = 0;

  virtual size_t size() const = 0;

  virtual void Push(const std::vector<int> &values, int node) = 0;

  virtual int Pop() = 0;

  virtual bool IsEmpty() const = 0;
};

} // namespace pplanner

#endif // OPEN_LIST_IMPL_H_
