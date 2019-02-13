#ifndef RO_OPEN_LIST_IMPL_H_
#define RO_OPEN_LIST_IMPL_H_

#include <random>

namespace pplanner {

template<typename T>
class ROOpenListImpl : public OpenListImpl<T> {
 public:
  ROOpenListImpl() : size_(0) {
    std::random_device rng;
    engine_ = std::mt19937(rng());
  }

  ~ROOpenListImpl() {}

  std::size_t size() const override { return size_; }

  void Push(const std::vector<int> &values, T node) override {
    buckets_[values].push_back(node);
    ++size_;
  }

  T Pop() override {
    auto it = buckets_.begin();
    auto &bucket = it->second;
    std::uint32_t r = engine_();
    std::uint32_t index = r % bucket.size();
    auto result = bucket[index];
    bucket[index] = bucket.back();
    bucket.pop_back();
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

  T PopWorst() override {
    auto it = buckets_.rbegin();
    auto &bucket = it->second;
    std::uint32_t r = engine_();
    std::uint32_t index = r % bucket.size();
    auto result = bucket[index];
    bucket[index] = bucket.back();
    bucket.pop_back();
    if (bucket.empty()) buckets_.erase(it->first);
    --size_;

    return result;
  }

 private:
  std::size_t size_;
  std::map<std::vector<int>, std::vector<T> > buckets_;
  std::mt19937 engine_;
};

} // namespace pplanner

#endif // RO_OPEN_LIST_IMPL_H_
