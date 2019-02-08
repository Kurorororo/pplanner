#ifndef PREFERRED_PLUS_OPEN_LIST_H_
#define PREFERRED_PLUS_OPEN_LIST_H_

#include <cassert>

#include <memory>
#include <string>
#include <vector>

#include "evaluator.h"
#include "open_lists/single_open_list.h"

namespace pplanner {

template<typename T>
class PreferredPlusOpenList : public SingleOpenList<T> {
 public:
  PreferredPlusOpenList() : SingleOpenList<T>(), weight_(1) {}

  explicit PreferredPlusOpenList(const std::string &tie_breaking)
      : SingleOpenList<T>(tie_breaking), weight_(1) {}

  PreferredPlusOpenList(
      const std::string &tie_breaking,
      const std::vector<std::shared_ptr<Evaluator> > &evaluators)
        : SingleOpenList<T>(tie_breaking, evaluators), weight_(1) {}

  ~PreferredPlusOpenList() {}

  void Push(std::vector<int> &values, T node, bool preferred) override {
    if (!preferred) values[0] += weight_;

    SingleOpenList<T>::Push(values, node, preferred);
  }

  void Boost() override {}

 private:
  int weight_;
};

} // namespace pplanner

#endif // PREFERRED_PLUS_OPEN_LIST_H_
