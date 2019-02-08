#ifndef FIFO_OPEN_LIST_IMPL_H_
#define FIFO_OPEN_LIST_IMPL_H_

#include <map>
#include <deque>

#include "open_lists/open_list_impl.h"

namespace pplanner {

template<typename T>
class FIFOOpenListImpl : public OpenListImpl<T> {
 public:
  FIFOOpenListImpl() : size_(0) {}

  ~FIFOOpenListImpl() {}

  std::size_t size() const override { return size_; }

  void Push(const std::vector<int> &values, T node) override {
    buckets_[values].push_back(node);
    ++size_;
  }

  T Pop() override {
    auto it = buckets_.begin();
    auto &bucket = it->second;
    auto result = bucket.front();
    bucket.pop_front();
    if (bucket.empty()) buckets_.erase(it);
    --size_;

    return result;
  }

  bool IsEmpty() const override { return size_ == 0; }

  int MinimumValue(int i) const override { return buckets_.begin()->first[i]; }

  const std::vector<int>& MinimumValues() const override {
    return buckets_.begin()->first;
  }

  void Clear() override { buckets_.clear(); }

 private:
  std::size_t size_;
  std::map<std::vector<int>, std::deque<T> > buckets_;
};

} // namespace pplanner

#endif // FIFO_OPEN_LIST_IMPL_H_
