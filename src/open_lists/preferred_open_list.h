#ifndef PREFERRED_OPEN_LIST_H_
#define PREFERRED_OPEN_LIST_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "open_list.h"
#include "open_lists/open_list_impl.h"
#include "open_lists/open_list_impl_factory.h"

namespace pplanner {

template<typename T>
class PreferredOpenList : public OpenList<T> {
 public:
  PreferredOpenList() : boost_(0) {}

  PreferredOpenList(const std::string &tie_breaking, int boost=0)
      : boost_(boost) { Init(tie_breaking); }

  ~PreferredOpenList() {}

  std::size_t size() const override { return lists_[0]->size(); }

  void Push(const std::vector<int> &values, T node, bool preferred) override {
    lists_[0]->Push(values, node);
    if (preferred) lists_[1]->Push(values, node);
  }

  T Pop() override;

  bool IsEmpty() const override {
    return lists_[0]->IsEmpty() && lists_[1]->IsEmpty();
  }

  int MinimumValue(int i) const override { return lists_[0]->MinimumValue(i); }

  const std::vector<int>& MinimumValues() const override {
    return lists_[0]->MinimumValues();
  }

  void Clear() override {
    lists_[0]->Clear();
    lists_[1]->Clear();
  }

  void Boost() override { priorities_[1] += boost_; }

  T PopWorst() override;

 private:
  void Init(const std::string &tie_breaking) {
    lists_[0] = OpenListImplFactory<T>(tie_breaking);
    lists_[1] = OpenListImplFactory<T>(tie_breaking);

    priorities_[0] = 0;
    priorities_[1] = 0;
  }

  int boost_;
  std::vector<int> values_;
  std::array<int, 2> priorities_;
  std::array<std::shared_ptr<OpenListImpl<T> >, 2> lists_;
};

template<typename T>
T PreferredOpenList<T>::Pop() {
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

template<typename T>
T PreferredOpenList<T>::PopWorst() {
  if (lists_[0]->IsEmpty())
    return lists_[1]->PopWorst();

  if (lists_[1]->IsEmpty())
    return lists_[0]->PopWorst();

  int arg_max = priorities_[0] > priorities_[1] ? 0 : 1;

  return lists_[1 - arg_max]->Pop();
}

} // namespace pplanner

#endif // PREFERRED_OPEN_LIST_H_
