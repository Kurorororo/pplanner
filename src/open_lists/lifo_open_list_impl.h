#ifndef LIFO_OPEN_LIST_IMPL_H_
#define LIFO_OPEN_LIST_IMPL_H_

#include <map>
#include <stack>

#include "open_lists/open_list_impl.h"

namespace pplanner {

class LIFOOpenListImpl : public OpenListImpl {
 public:
  LIFOOpenListImpl() : size_(0) {}

  ~LIFOOpenListImpl() {}

  size_t size() const override { return size_; }

  void Push(const std::vector<int> &values, int node) override {
    buckets_[values].push(node);
    ++size_;
  }

  int Pop() override {
    auto it = buckets_.begin();
    auto &bucket = it->second;
    auto result = bucket.top();
    bucket.pop();
    if (bucket.empty()) buckets_.erase(it);
    --size_;

    return result;
  }

  bool IsEmpty() const override { return size_ == 0; }

  int MinimumValue(int i) const override { return buckets_.begin()->first[i]; }

 private:
  size_t size_;
  std::map<std::vector<int>, std::stack<int> > buckets_;
};

} // namespace pplanner

#endif // LIFO_OPEN_LIST_IMPL_H_
