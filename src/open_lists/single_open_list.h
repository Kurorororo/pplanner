#ifndef SINGLE_OPEN_LIST_H_
#define SINGLE_OPEN_LIST_H_

#include <cassert>

#include <memory>
#include <string>
#include <vector>

#include "open_list.h"
#include "utils/priority_queue.h"

namespace pplanner {

template <typename T = std::vector<int>, typename U = int>
class SingleOpenList : public OpenList<T, U> {
 public:
  SingleOpenList() : list_(nullptr) {}

  explicit SingleOpenList(const std::string &tie_breaking)
      : list_(PriorityQueueFactory<T, U>(tie_breaking)) {}

  ~SingleOpenList() {}

  std::size_t size() const override { return list_->size(); }

  void Push(T values, U node, bool preferred) override {
    assert(list_ != nullptr);

    list_->Push(values, node);
  }

  U Pop() override {
    assert(list_ != nullptr);

    return list_->Pop();
  }

  bool IsEmpty() const override {
    assert(list_ != nullptr);

    return list_->IsEmpty();
  }

  const T &MinimumValue() const override { return list_->MinimumValue(); }

  void Clear() override { list_->Clear(); }

  void Boost() override {}

 private:
  std::shared_ptr<PriorityQueue<T, U> > list_;
};

}  // namespace pplanner

#endif  // SINGLE_OPEN_LIST_H_
