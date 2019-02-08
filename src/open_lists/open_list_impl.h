#ifndef OPEN_LIST_IMPL_H_
#define OPEN_LIST_IMPL_H_

#include <vector>

namespace pplanner {

template<typename T>
class OpenListImpl {
 public:
  virtual ~OpenListImpl() = 0;

  virtual std::size_t size() const = 0;

  virtual void Push(const std::vector<int> &values, T node) = 0;

  virtual T Pop() = 0;

  virtual bool IsEmpty() const = 0;

  virtual int MinimumValue(int i) const = 0;

  virtual const std::vector<int>& MinimumValues() const = 0;

  virtual void Clear() = 0;
};

template<typename T>
OpenListImpl<T>::~OpenListImpl() {}

} // namespace pplanner

#endif // OPEN_LIST_IMPL_H_
