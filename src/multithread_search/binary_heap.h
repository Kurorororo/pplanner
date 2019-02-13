#ifndef BINARY_HEAP_H_
#define BINARY_HEAP_H_

#include <functional>
#include <vector>

namespace pplanner {

template<class T, class Compare = std::less<T> >
class BinaryHeap() {
 public:
  void Push();

  T Pop();

  bool IsEmpty();

  void Delete(const T &element);

 private:
  Compare compare_;
  std::vector<T> heap_;
};

} // namespace pplanner

#endif // BINARY_HEAP_H_
