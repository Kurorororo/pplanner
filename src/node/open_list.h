#ifndef OPEN_LIST_H_
#define OPEN_LIST_H_

#include <deque>
#include <limits>
#include <map>
#include <queue>
#include <vector>

namespace rwls {

template<typename T, typename U>
class OpenList {
 public:
  OpenList() : size_(0) {}

  explicit OpenList(const OpenList &list) {
    buckets_ = list.buckets_;
    size_ = list.size_;
  }

  ~OpenList() {}

  inline void Push(T value, U ptr) {
    buckets_[value].push_back(ptr);
    ++size_;
  }

  inline U Top() {
    return buckets_.begin()->second.front();
  }

  inline U Pop() {
    auto it = buckets_.begin();
    auto &bucket = it->second;
    auto result = bucket.front();
    bucket.pop_front();
    if (bucket.empty()) buckets_.erase(it);
    --size_;
    return result;
  }

  inline bool IsEmpty() {
    return size_ == 0;
  }

  inline size_t Size() {
    return size_;
  }

 private:
  std::map<T, std::deque<U> > buckets_;
  size_t size_;
};

template <typename T, typename U>
class AlternateOpenList {
 public:
  AlternateOpenList() : n_lists_(0) {}

  explicit AlternateOpenList(int n_lists) : n_lists_(n_lists) {
    priorities_.resize(n_lists, 0);
    lists_.resize(n_lists);
  }

  ~AlternateOpenList() {}

  void Push(T value, U ptr) {
    for (int i=0; i<n_lists_; ++i)
      lists_[i].Push(value, ptr);
  }

  void Push(int i, T value, U ptr) {
    lists_[i].Push(value, ptr);
  }

  T Top() {
    int arg_max = -1;
    int max = std::numeric_limits<int>::min();

    for (int i=0; i<n_lists_; ++i) {
      if (priorities_[i] > max && !lists_[i].IsEmpty()) {
        max = priorities_[i];
        arg_max = i;
      }
    }

    return lists_[arg_max].Top();
  }

  T Pop() {
    int arg_max = -1;
    int max = std::numeric_limits<int>::min();

    for (int i=0; i<n_lists_; ++i) {
      if (priorities_[i] > max && !lists_[i].IsEmpty()) {
        max = priorities_[i];
        arg_max = i;
      }
    }

    --priorities_[arg_max];

    return lists_[arg_max].Pop();
  }

  bool IsEmpty() {
    for (auto it = lists_.begin(); it != lists_.end(); ++it)
      if (!it->IsEmpty()) return false;

    return true;
  }

  void Boost(int i, int n_boost) {
    priorities_[i] += n_boost;
  }

 private:
  int n_lists_;
  std::vector<int> priorities_;
  std::vector<OpenList<T, U> > lists_;
};
}

#endif // OPEN_LIST_H_
