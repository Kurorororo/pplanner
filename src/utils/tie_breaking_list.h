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

  bool IsEmpty() const { return list_.empty(); }

  void Push(const T& a) { list_.push_back(a); }

  T Pop() {
    auto f = list_.front();
    list_.pop_front();

    return f;
  }

 private:
  std::deque<T> list_;
};

template <typename T>
class LIFOList {
 public:
  LIFOList() {}

  bool IsEmpty() const { return list_.empty(); }

  void Push(const T& a) { list_.push_back(a); }

  T Pop() {
    auto f = list_.back();
    list_.pop_back();

    return f;
  }

 private:
  std::deque<T> list_;
};

template <typename T>
class ROList {
 public:
  ROList() {
    std::random_device rng;
    engine_ = std::mt19937(rng());
  }

  bool IsEmpty() const { return list_.empty(); }

  void Push(const T& a) { list_.push_back(a); }

  T Pop() {
    std::uint32_t r = engine_();
    std::uint32_t index = r % list_.size();
    auto f = list_[index];
    list_[index] = list_.back();
    list_.pop_back();

    return f;
  }

 private:
  std::vector<T> list_;
  std::mt19937 engine_;
};

}  // namespace pplanner

#endif  // TIEBREKING_LIST_H_