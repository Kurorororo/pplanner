#ifndef PREFERRED_PLUS_OPEN_LIST_H_
#define PREFERRED_PLUS_OPEN_LIST_H_

#include <cassert>

#include <memory>
#include <string>
#include <vector>

#include "open_list.h"
#include "open_lists/single_open_list.h"

namespace pplanner {

template <typename T = std::vector<int>, typename U = int>
class PreferredPlusOpenList : public SingleOpenList<T, U> {
 public:
  PreferredPlusOpenList() : SingleOpenList<T>(), weight_(1) {}

  explicit PreferredPlusOpenList(const std::string &tie_breaking)
      : SingleOpenList<T, U>(tie_breaking), weight_(1) {}

  ~PreferredPlusOpenList() {}

  void Push(T values, U node, bool preferred) override {
    auto v = values;

    if (!preferred) v[0] += weight_;

    SingleOpenList<T, U>::Push(v, node, preferred);
  }

  void Boost() override {}

 private:
  int weight_;
};

}  // namespace pplanner

#endif  // PREFERRED_PLUS_OPEN_LIST_H_
