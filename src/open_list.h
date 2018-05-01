#ifndef OPEN_LIST_H_
#define OPEN_LIST_H_

namespace pplanner {

class OpenList {
 public:
  virtual ~OpenList() = 0;

  virtual void Push(const std::vector<int> &values, int node, bool preferred)
    = 0;

  virtual int Push(const std::vector<int> &state, int node, bool preferred) = 0;

  virtual int Pop() = 0;

  virtual bool IsEmpty() const = 0;

  virtual void Boost() = 0;
};

} // namespace pplanner

#endif // OPEN_LIST_H_
