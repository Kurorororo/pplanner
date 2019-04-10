#ifndef OPEN_LIST_H_
#define OPEN_LIST_H_

#include <vector>

namespace pplanner {

template <typename T = std::vector<int>, typename U = int>
class OpenList {
 public:
  virtual ~OpenList() = 0;

  virtual std::size_t size() const = 0;

  virtual void Push(T values, U node, bool preferred) = 0;

  virtual U Pop() = 0;

  virtual bool IsEmpty() const = 0;

  virtual const T &MinimumValue() const = 0;

  virtual void Clear() = 0;

  virtual void Boost() = 0;
};

template <typename T, typename U>
OpenList<T, U>::~OpenList() {}

}  // namespace pplanner

#endif  // OPEN_LIST_H_
