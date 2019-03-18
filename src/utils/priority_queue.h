#ifndef INT_PRIORITY_QUEUE_H_
#define INT_PRIORITY_QUEUE_H_

#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "tie_breaking_list.h"

namespace pplanner {

template <typename T, typename U>
class PriorityQueue {
 public:
  virtual ~PriorityQueue() = 0;

  virtual std::size_t size() const = 0;

  virtual bool IsEmpty() const = 0;

  virtual void Push(T priority, U entry) = 0;

  virtual const T& MinimumValue() const = 0;

  virtual U Pop() = 0;

  virtual void Clear() = 0;
};

template <typename T, typename U>
PriorityQueue<T, U>::~PriorityQueue() {}

template <typename T, typename U>
class CppPriorityQueue : public PriorityQueue<T, U> {
 public:
  CppPriorityQueue() {}

  ~CppPriorityQueue() {}

  std::size_t size() const override { return q_.size(); }

  bool IsEmpty() const override { return q_.empty(); }

  void Push(T priority, U entry) override {
    q_.push(std::make_pair(priority, entry));
  }

  const T& MinimumValue() const override { return q_.top().first; }

  U Pop() override {
    auto top = q_.top();
    q_.pop();
    return top.second;
  }

  void Clear() override {
    auto new_q = PQueue();
    q_.swap(new_q);
  }

 private:
  struct FirstGreater {
    bool operator()(const std::pair<T, U>& a, const std::pair<T, U>& b) {
      return a.first > b.first;
    }
  };

  using PQueue =
      std::priority_queue<std::pair<T, U>, std::vector<std::pair<T, U> >,
                          FirstGreater>;
  PQueue q_;
};

template <typename T, typename U, typename V>
class TieBreakingPriorityQueue : public PriorityQueue<T, U> {
 public:
  TieBreakingPriorityQueue() : size_(0) {}

  ~TieBreakingPriorityQueue() {}

  std::size_t size() const override { return size_; }

  bool IsEmpty() const override { return size_ == 0; }

  void Push(T priority, U entry) override {
    buckets_[std::move(priority)].Push(entry);
    ++size_;
  }

  const T& MinimumValue() const override { return buckets_.begin()->first; }

  U Pop() override {
    auto it = buckets_.begin();
    auto& bucket = it->second;
    auto result = bucket.Pop();
    if (bucket.IsEmpty()) buckets_.erase(it);
    --size_;

    return result;
  }

  void Clear() override {
    size_ = 0;
    buckets_.clear();
  }

 private:
  std::size_t size_;
  std::map<T, V> buckets_;
};

template <typename U, typename V>
class VectorPriorityQueue : public PriorityQueue<int, U> {
 public:
  VectorPriorityQueue() : size_(0), minimum_value_(-1), buckets_(100) {}

  ~VectorPriorityQueue() {}

  std::size_t size() const override { return size_; }

  bool IsEmpty() const override { return size_ == 0; }

  void Push(int priority, U entry) override {
    if (priority < minimum_value_ || minimum_value_ == -1)
      minimum_value_ = priority;

    if (static_cast<int>(buckets_.size()) <= priority)
      buckets_.resize(priority + 1);

    buckets_[priority].Push(entry);
    ++size_;
  }

  const int& MinimumValue() const override { return minimum_value_; }

  U Pop() override {
    auto result = buckets_[minimum_value_].Pop();
    --size_;

    if (buckets_[minimum_value_].IsEmpty()) {
      if (size_ > 0) {
        for (int i = minimum_value_ + 1, n = buckets_.size(); i < n; ++i) {
          if (!buckets_[i].IsEmpty()) {
            minimum_value_ = i;
            break;
          }
        }
      } else {
        minimum_value_ = -1;
      }
    }

    return result;
  }

  void Clear() override {
    size_ = 0;
    minimum_value_ = -1;

    for (auto& bucket : buckets_) bucket.Clear();
  }

 private:
  std::size_t size_;
  int minimum_value_;
  std::vector<V> buckets_;
};

template <typename T, typename U>
std::unique_ptr<PriorityQueue<T, U> > PriorityQueueFactory(
    const std::string& tie_break) {
  if (tie_break == "cpp") return std::make_unique<CppPriorityQueue<T, U> >();

  if (tie_break == "fifo")
    return std::make_unique<TieBreakingPriorityQueue<T, U, FIFOList<U> > >();

  if (tie_break == "lifo")
    return std::make_unique<TieBreakingPriorityQueue<T, U, LIFOList<U> > >();

  if (tie_break == "ro")
    return std::make_unique<TieBreakingPriorityQueue<T, U, ROList<U> > >();

  return std::make_unique<CppPriorityQueue<T, U> >();
}

template <typename U>
std::unique_ptr<PriorityQueue<int, U> > VectorPriorityQueueFactory(
    const std::string& tie_break) {
  if (tie_break == "fifo")
    return std::make_unique<VectorPriorityQueue<U, FIFOList<U> > >();

  if (tie_break == "lifo")
    return std::make_unique<VectorPriorityQueue<U, LIFOList<U> > >();

  if (tie_break == "ro")
    return std::make_unique<VectorPriorityQueue<U, ROList<U> > >();

  return std::make_unique<VectorPriorityQueue<U, FIFOList<U> > >();
}

}  // namespace pplanner

#endif  // INT_PRIORITY_QUEUE_H_