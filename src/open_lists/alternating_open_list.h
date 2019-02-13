#ifndef ALTERNATING_OPEN_LIST_H_
#define ALTERNATING_OPEN_LIST_H_

#include <cassert>

#include <memory>
#include <string>
#include <vector>

#include "open_list.h"
#include "open_lists/open_list_impl.h"
#include "open_lists/open_list_impl_factory.h"

namespace pplanner {

template<typename T>
class AlternatingOpenList : public OpenList<T> {
 public:
  AlternatingOpenList() {}

  explicit AlternatingOpenList(int n, const std::string &tie_breaking)
      : lists_(n, OpenListImplFactory<T>(tie_breaking)) {}

  ~AlternatingOpenList() {}

  std::size_t size() const override;

  void Push(const std::vector<int> &values, T node, bool preferred) override;

  T Pop() override;

  bool IsEmpty() const override;

  int MinimumValue(int i) const override { return lists_[i]->MinimumValue(0); }

  const std::vector<int>& MinimumValues() const override;

  void Clear() override;

  void Boost() override {}

  T PopWorst() override;

 private:
  int idx_;
  std::vector<std::shared_ptr<OpenListImpl<T> > > lists_;
};

template<typename T>
std::size_t AlternatingOpenList<T>::size() const {
  std::size_t size = 0;

  for (int i = 0, n = lists_.size(); i < n; ++i)
    size += lists_[i]->size();

  return size;
}

template<typename T>
void AlternatingOpenList<T>::Push(const std::vector<int> &values, T node,
                                  bool preferred) {
  for (int i = 0, n = values.size(); i < n; ++i)
    lists_[i]->Push(std::vector<int>{values[i]}, node);
}

template<typename T>
T AlternatingOpenList<T>::Pop() {
  for (int i = 0, n = lists_.size(); i < n; ++i) {
    if (!lists_[idx_]->IsEmpty())
      break;

    idx_ = (idx_ + 1) % lists_.size();
  }

  int node = lists_[idx_]->Pop();
  idx_ = (idx_ + 1) % lists_.size();

  return node;
}

template<typename T>
bool AlternatingOpenList<T>::IsEmpty() const {
  for (int i = 0, n = lists_.size(); i < n; ++i)
    if (!lists_[i]->IsEmpty())
      return false;

  return true;
}

template<typename T>
const std::vector<int>& AlternatingOpenList<T>::MinimumValues() const {
  thread_local std::vector<int> values;

  values.clear();

  for (int i = 0, n = values.size(); i < n; ++i)
    values.push_back(lists_[i]->MinimumValue(0));

  return values;
}

template<typename T>
void AlternatingOpenList<T>::Clear() {
  for (int i = 0, n = lists_.size(); i < n; ++i)
    lists_[idx_]->Clear();
}

template<typename T>
T AlternatingOpenList<T>::PopWorst() {
  return lists_[idx_]->PopWorst();
}

} // namespace pplanner

#endif // ALTERNATING_OPEN_LIST_H_
