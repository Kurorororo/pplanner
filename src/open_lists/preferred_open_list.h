#ifndef PREFERRED_OPEN_LIST_H_
#define PREFERRED_OPEN_LIST_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "open_list.h"
#include "utils/priority_queue.h"

namespace pplanner {

template <typename T = std::vector<int>, typename U = int>
class PreferredOpenList : public OpenList<T, U> {
 public:
  PreferredOpenList() : boost_(0) {}

  PreferredOpenList(const std::string &tie_breaking, int boost = 0)
      : boost_(boost) {
    Init(tie_breaking);
  }

  ~PreferredOpenList() {}

  std::size_t size() const override { return lists_[0]->size(); }

  void Push(T values, U node, bool preferred) override {
    lists_[0]->Push(values, node);
    if (preferred) lists_[1]->Push(values, node);
  }

  U Top() const override;

  U Pop() override;

  bool IsEmpty() const override {
    return lists_[0]->IsEmpty() && lists_[1]->IsEmpty();
  }

  const T &MinimumValue() const override {
    int idx = lists_[0]->MinimumValue() < lists_[1]->MinimumValue() ? 0 : 1;

    return lists_[idx]->MinimumValue();
  }

  void Clear() override {
    lists_[0]->Clear();
    lists_[1]->Clear();
  }

  void Boost() override { priorities_[1] += boost_; }

 private:
  void Init(const std::string &tie_breaking) {
    lists_[0] = PriorityQueueFactory<T, U>(tie_breaking);
    lists_[1] = PriorityQueueFactory<T, U>(tie_breaking);

    priorities_[0] = 0;
    priorities_[1] = 0;
  }

  int boost_;
  std::vector<int> values_;
  std::array<int, 2> priorities_;
  std::array<std::shared_ptr<PriorityQueue<T, U> >, 2> lists_;
};

template <typename T, typename U>
U PreferredOpenList<T, U>::Top() const {
  if (lists_[0]->IsEmpty()) return lists_[1]->Top();

  if (lists_[1]->IsEmpty()) return lists_[0]->Top();

  int arg_max = priorities_[0] > priorities_[1] ? 0 : 1;

  return lists_[arg_max]->Top();
}

template <typename T, typename U>
U PreferredOpenList<T, U>::Pop() {
  if (lists_[0]->IsEmpty()) {
    --priorities_[1];

    return lists_[1]->Pop();
  }

  if (lists_[1]->IsEmpty()) {
    --priorities_[0];

    return lists_[0]->Pop();
  }

  int arg_max = priorities_[0] > priorities_[1] ? 0 : 1;
  --priorities_[arg_max];

  return lists_[arg_max]->Pop();
}

}  // namespace pplanner

#endif  // PREFERRED_OPEN_LIST_H_
