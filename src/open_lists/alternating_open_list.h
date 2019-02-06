#ifndef ALTERNATING_OPEN_LIST_H_
#define ALTERNATING_OPEN_LIST_H_

#include <cassert>

#include <memory>
#include <string>
#include <vector>

#include "evaluator.h"
#include "open_list.h"
#include "open_lists/open_list_impl.h"
#include "open_lists/open_list_impl_factory.h"

namespace pplanner {

class AlternatingOpenList : public OpenList {
 public:
  AlternatingOpenList() {}

  explicit AlternatingOpenList(int n, const std::string &tie_breaking)
      : lists_(n, OpenListImplFactory(tie_breaking)) {}

  AlternatingOpenList(
      const std::string &tie_breaking,
      const std::vector<std::shared_ptr<Evaluator> > &evaluators)
      : idx_(0),
        lists_(evaluators.size(), OpenListImplFactory(tie_breaking)),
        evaluators_(evaluators) {}

  ~AlternatingOpenList() {}

  std::size_t size() const override;

  void Push(std::vector<int> &values, int node, bool preferred) override;

  int EvaluateAndPush(const std::vector<int> &state, int node, bool preferred)
    override;

  int Pop() override;

  bool IsEmpty() const override;

  int MinimumValue(int i) const override { return lists_[i]->MinimumValue(0); }

  const std::vector<int>& MinimumValues() const override;

  void Clear() override;

  void Boost() override {}

 private:
  int idx_;
  std::vector<std::shared_ptr<OpenListImpl> > lists_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
};

} // namespace pplanner

#endif // ALTERNATING_OPEN_LIST_H_
