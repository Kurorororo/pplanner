#ifndef OPEN_LIST_H_
#define OPEN_LIST_H_

#include <vector>

namespace pplanner {

class OpenList {
 public:
  virtual ~OpenList() = 0;

  virtual size_t size() const = 0;

  virtual void Push(std::vector<int> &values, int node, bool preferred) = 0;

  virtual int EvaluateAndPush(const std::vector<int> &state, int node,
                              bool preferred) = 0;

  virtual int Pop() = 0;

  virtual bool IsEmpty() const = 0;

  virtual void Boost() = 0;
};

} // namespace pplanner

#endif // OPEN_LIST_H_
