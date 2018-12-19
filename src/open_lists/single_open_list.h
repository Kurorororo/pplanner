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

class SingleOpenList : public OpenList {
 public:
  SingleOpenList() : list_(nullptr) {}

  explicit SingleOpenList(const std::string &tie_breaking)
      : list_(OpenListImplFactory(tie_breaking)) {}

  SingleOpenList(const std::string &tie_breaking,
                 const std::vector<std::shared_ptr<Evaluator> > &evaluators)
      : list_(OpenListImplFactory(tie_breaking)), evaluators_(evaluators) {}

  ~SingleOpenList() {}

  std::size_t size() const override { return list_->size(); }

  void Push(std::vector<int> &values, int node, bool preferred) override {
    assert(list_ != nullptr);

    list_->Push(values, node);
  }

  int EvaluateAndPush(const std::vector<int> &state, int node, bool preferred)
    override;

  int Pop() override {
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
  std::unique_ptr<OpenListImpl> list_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
};

} // namespace pplanner

#endif // SINGLE_OPEN_LIST_H_
