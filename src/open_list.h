#ifndef OPEN_LIST_H_
#define OPEN_LIST_H_

#include <memory>
#include <string>
#include <vector>

#include "evaluator.h"
#include "open_lists/open_list_impl.h"
#include "open_lists/open_list_impl_factory.h"

namespace pplanner {

class OpenList {
 public:
  OpenList() : list_(nullptr), evaluators_(nullptr) {};

  explicit OpenList(const std::string &tie_breaking)
      : list_(OpenListImplFactory(tie_breaking)), evaluators_(nullptr) {}

  OpenList(const std::string &tie_breaking,
           const std::vector<std::shared_ptr<Evaluator> > &evaluators)
      : list_(OpenListImplFactory(tie_breaking)), evaluators_(evaluators_) {}

  void Push(const vector<int> &values, int node) { list_->Push(values, node); }

  int Push(const std::vector<int> &state, int node);

  int Pop() { return list_->Pop(); }

  bool IsEmpty() const { return list_->IsEmpty(); }

 private:
  void Init(const std::vector<std::string> &evaluator_names);

  std::vector<int> values_;
  std::unique_ptr<OpenListImpl> list_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
};

} // namespace pplanner

#endif // OPEN_LIST_H_
