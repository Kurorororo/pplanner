#ifndef SINGLE_OPEN_LIST_H_
#define SINGLE_OPEN_LIST_H_

#include <cassert>

#include <memory>
#include <string>
#include <vector>

#include "evaluator.h"
#include "open_list.h"
#include "open_lists/open_list_impl.h"
#include "open_lists/open_list_impl_factory.h"

namespace pplanner {

template<typename T>
class SingleOpenList : public OpenList<T> {
 public:
  SingleOpenList() : list_(nullptr) {}

  explicit SingleOpenList(const std::string &tie_breaking)
      : list_(OpenListImplFactory<T>(tie_breaking)) {}

  SingleOpenList(const std::string &tie_breaking,
                 const std::vector<std::shared_ptr<Evaluator> > &evaluators)
      : list_(OpenListImplFactory<T>(tie_breaking)), evaluators_(evaluators) {}

  ~SingleOpenList() {}

  std::size_t size() const override { return list_->size(); }

  void Push(std::vector<int> &values, T node, bool preferred) override {
    assert(list_ != nullptr);

    list_->Push(values, node);
  }

  int EvaluateAndPush(const std::vector<int> &state, T node, bool preferred)
    override {
    values_.clear();

    for (auto evaluator : evaluators_) {
      int value = evaluator->Evaluate(state, node);
      if (value == -1) return value;
      values_.push_back(value);
    }

    Push(values_, node, preferred);

    return values_[0];
  }

  T Pop() override {
    assert(list_ != nullptr);

    return list_->Pop();
  }

  bool IsEmpty() const override {
    assert(list_ != nullptr);

    return list_->IsEmpty();
  }

  int MinimumValue(int i) const override { return list_->MinimumValue(i); }

  const std::vector<int>& MinimumValues() const override {
    return list_->MinimumValues();
  }

  void Clear() override { list_->Clear(); }

  void Boost() override {}

 private:
  std::vector<int> values_;
  std::shared_ptr<OpenListImpl<T> > list_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
};

} // namespace pplanner

#endif // SINGLE_OPEN_LIST_H_
