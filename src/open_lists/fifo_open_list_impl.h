#ifndef FIFO_OPEN_LIST_IMPL_H_
#define FIFO_OPEN_LIST_IMPL_H_

#include <map>
#include <deque>

#include "open_list/open_list_impl.h"

namespace pplanner {

class FIFOOpenListImpl : OpenListImpl {
 public:
  FIFOOpenListImpl() : size_(0) {}

  ~FIFOOpenListImpl() {}

  void Push(std::vector<int> values, int node) override {
    buckets_[value].push_back(node);
    ++size_;
  }

  int Pop() override {
    auto it = buckets_.begin();
    auto &bucket = it->second;
    auto result = bucket.front();
    bucket.pop_front();
    if (bucket.empty()) buckets_.erase(it);
    --size_;

    return result;
  }

  bool IsEmpty() const override { return size_ == 0; }

 private:
  size_t size_;
  std::map<int, std::deque<int> > buckets_;
};

} // namespace pplanner

#endif // FIFO_OPEN_LIST_IMPL_H_
