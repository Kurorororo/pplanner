#ifndef BEST_FIRST_BUFFER_H_
#define BEST_FIRST_BUFFER_H_

#include <map>
#include <vector>

namespace pplanner {

class BestFirstBuffer {
 public:
  BestFirstBuffer() : size_(0) {}

  ~BestFirstBuffer() {}

  unsigned char* Extend(int h) {
  }


 private:
  size_t size_;
  std::map<int, std::vector<int> > buffers_;
};

} // namespace pplanner

#endif // BEST_FIRST_BUFFER_H_
