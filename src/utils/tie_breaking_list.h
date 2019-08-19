#ifndef TIEBREKING_LIST_H_
#define TIEBREKING_LIST_H_

#include <deque>
#include <random>
#include <vector>

namespace pplanner {

template <typename T>
class FIFOList {
 public:
  FIFOList() {}

  std::size_t size() const { return list_.size(); }

  bool IsEmpty() const { return list_.empty(); }

  void Push(const T& a) { list_.push_back(a); }

  T Top() const { return list_.front(); }

  T Pop() {
    auto f = list_.front();
    list_.pop_front();

    return f;
  }

  void Clear() { list_.clear(); }

 private:
  std::deque<T> list_;
};

template <typename T>
class LIFOList {
 public:
  LIFOList() {}

  std::size_t size() const { return list_.size(); }

  bool IsEmpty() const { return list_.empty(); }

  void Push(const T& a) { list_.push_back(a); }

  T Top() const { return list_.back(); }

  T Pop() {
    auto f = list_.back();
    list_.pop_back();

    return f;
  }

  void Clear() { list_.clear(); }

 private:
  std::deque<T> list_;
};

template <typename T>
class ROList {
 public:
  ROList() {
    std::random_device rng;
    engine_ = std::mt19937(rng());
    r_ = engine_();
  }

  std::size_t size() const { return list_.size(); }

  bool IsEmpty() const { return list_.empty(); }

  void Push(const T& a) { list_.push_back(a); }

  T Top() const { return list_[r_ % list_.size()]; }

  T Pop() {
    std::uint32_t index = r_ % list_.size();
    auto f = list_[index];
    list_[index] = list_.back();
    list_.pop_back();
    r_ = engine_();

    return f;
  }

  void Clear() { list_.clear(); }

 private:
  std::vector<T> list_;
  std::mt19937 engine_;
  std::uint32_t r_;
};

}  // namespace pplanner

#endif  // TIEBREKING_LIST_H_