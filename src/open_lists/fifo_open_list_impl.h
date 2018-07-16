#ifndef FIFO_OPEN_LIST_IMPL_H_
#define FIFO_OPEN_LIST_IMPL_H_

#include <map>
#include <deque>

#include "open_lists/open_list_impl.h"

namespace pplanner {

class FIFOOpenListImpl : public OpenListImpl {
 public:
  FIFOOpenListImpl() : size_(0) {}

  ~FIFOOpenListImpl() {}

  size_t size() const override { return size_; }

  void Push(const std::vector<int> &values, int node) override {
    buckets_[values].push_back(node);
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
  std::map<std::vector<int>, std::deque<int> > buckets_;
};

} // namespace pplanner

#endif // FIFO_OPEN_LIST_IMPL_H_
