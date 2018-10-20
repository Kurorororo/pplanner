#ifndef BEST_FIRST_BUFFER_H_
#define BEST_FIRST_BUFFER_H_

#include <cstddef>

#include <map>
#include <vector>

namespace pplanner {

class BestFirstBuffer {
 public:
  BestFirstBuffer() {}

  ~BestFirstBuffer() {}

  std::map<std::vector<int>,
           std::vector<unsigned char> >::iterator MinBuffer() {
    return buffers_.begin();
  }

  std::map<std::vector<int>,
           std::vector<unsigned char> >::iterator BufferEnd() {
    return buffers_.end();
  }

  unsigned char* Extend(const std::vector<int> &values, size_t node_size) {
    if (buffers_[values].empty()) {
      buffers_[values].resize(values.size() * sizeof(int));
      memcpy(buffers_[values].data(), values.data(),
             values.size() * sizeof(int));
    }

    size_t old_size = buffers_[values].size();
    buffers_[values].resize(old_size + node_size);

    return buffers_[values].data() + old_size;
  }

  std::map<std::vector<int>, std::vector<unsigned char> >::iterator Erase(
      std::map<std::vector<int>, std::vector<unsigned char> >::iterator itr) {
    return buffers_.erase(itr);
  }

 private:
  std::map<std::vector<int>, std::vector<unsigned char> > buffers_;
};

} // namespace pplanner

#endif // BEST_FIRST_BUFFER_H_
