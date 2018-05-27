#ifndef PREFERRED_PLUS_OPEN_LIST_H_
#define PREFERRED_PLUS_OPEN_LIST_H_

#include <cassert>

#include <memory>
#include <string>
#include <vector>

#include "evaluator.h"
#include "open_lists/single_open_list.h"

namespace pplanner {

class PreferredPlusOpenList : public SingleOpenList {
 public:
  PreferredPlusOpenList() : SingleOpenList(), weight_(1) {}

  explicit PreferredPlusOpenList(const std::string &tie_breaking)
      : SingleOpenList(tie_breaking), weight_(1) {}

  PreferredPlusOpenList(
      const std::string &tie_breaking,
      const std::vector<std::shared_ptr<Evaluator> > &evaluators)
        : SingleOpenList(tie_breaking, evaluators), weight_(1) {}

  ~PreferredPlusOpenList() {}

  void Push(std::vector<int> &values, int node, bool preferred) override {
    if (!preferred) values[0] += weight_;

    SingleOpenList::Push(values, node, preferred);
  }

  void Boost() override {}

 private:
  int weight_;
};

} // namespace pplanner

#endif // PREFERRED_PLUS_OPEN_LIST_H_
