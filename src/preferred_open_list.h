#ifndef PREFERRED_OPEN_LIST_H_
#define PREFERRED_OPEN_LIST_H_

#include <memory>
#include <string>
#include <vector>

#include "evaluator.h"
#include "open_lists/open_list_impl.h"
#include "open_lists/open_list_impl_factory.h"

namespace pplanner {

class PreferredOpenList {
 public:
  PreferredOpenList() : boost_(1000), evaluators_(nullptr) {}

  PreferredOpenList(const std::string &tie_breaking, boost=1000)
      : boost_(boost), evaluators_(nullptr) { Init(tie_breaking); }

  PreferredOpenList(const std::string &tie_breaking,
                    const std::vector<std::string> &evaluators,
                    boost=1000)
      : evaluators_(evaluators), boost_(boost) { Init(tie_breaking); }

  void Push(const std::vector<int> &values, int node, bool preferred=false) {
    lists_[0]->Push(values, node);
    if (preferred) lists_[1]->Push(values, node);
  }

  int Push(const std::vector<int> &state, int node, bool preferred=false);

  int Pop();

  bool IsEmpty() const { return lists_[0]->IsEmpty() && lists_[1]->IsEmpty(); }

  void Boost() { priorities_[1] += boost_; }

 private:
  void Init(const std::string &tie_breaking) {
    lists_[0] = OpenListImplFactory(tie_breaking);
    lists_[1] = OpenListImplFactory(tie_breaking);

    priorities_[0] = 0;
    priorities_[1] = 0;
  }

  int boost_;
  std::array<int, 2> priorities_;
  std::array<std::unique_ptr<OpenList<T, int> >, 2> lists_;
  std::vector<std::shared_ptr<Evaluator> > evaluators_;
};

} // namespace pplanner

#endif // PREFERRED_OPEN_LIST_H_
