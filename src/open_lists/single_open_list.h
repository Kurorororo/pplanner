#ifndef SINGLE_OPEN_LIST_H_
#define SINGLE_OPEN_LIST_H_


#include <memory>
#include <string>
#include <vector>

#include "evaluator.h"
#include "open_list.h"
#include "open_lists/open_list_impl.h"
#include "open_lists/open_list_impl_factory.h"

namespace pplanner {

class SingleOpenList : OpenList {
 public:
  SingleOpenList() : list_(nullptr), evaluators_(nullptr) {};

  explicit SingleOpenList(const std::string &tie_breaking)
      : list_(OpenListImplFactory(tie_breaking)), evaluators_(nullptr) {}

  SingleOpenList(const std::string &tie_breaking,
                 const std::vector<std::shared_ptr<Evaluator> > &evaluators)
      : list_(OpenListImplFactory(tie_breaking)), evaluators_(evaluators) {}

  ~SingleOpenList();

  void Push(const vector<int> &values, int node, bool preferred) override {
    list_->Push(values, node, preferred);
  }

  int Push(const std::vector<int> &state, int node, bool preferred) override;

  int Pop() override { return list_->Pop(); }

  bool IsEmpty() const override { return list_->IsEmpty(); }

 private:
  std::vector<int> values_;
  std::unique_ptr<OpenListImpl> list_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
};

} // namespace pplanner

#endif // SINGLE_OPEN_LIST_H_
