#ifndef OPEN_LIST_H_
#define OPEN_LIST_H_

#include <vector>

namespace pplanner {

template<typename T = int >
class OpenList {
 public:
  virtual ~OpenList() = 0;

  virtual std::size_t size() const = 0;

  virtual void Push(std::vector<int> &values, T node, bool preferred) = 0;

  virtual int EvaluateAndPush(const std::vector<int> &state, T node,
                              bool preferred) = 0;

  virtual T Pop() = 0;

  virtual bool IsEmpty() const = 0;

  virtual int MinimumValue(int i) const = 0;

  virtual const std::vector<int>& MinimumValues() const = 0;

  virtual void Clear() = 0;

  virtual void Boost() = 0;
};

template<typename T>
OpenList<T>::~OpenList() {}

} // namespace pplanner

#endif // OPEN_LIST_H_
