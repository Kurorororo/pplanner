#ifndef NONE_H_
#define NONE_H_

#include <unordered_set>
#include <vector>

#include "domain/domain.h"
#include "node/node.h"

namespace rwls {

struct None {
  void Initialize(const Domain &domain) {}

  int operator()(const Node &node, const Domain &domain,
                 const std::vector<int> &applicable,
                 std::unordered_set<int> &preferred) {
    return 0;
  }
};

} // namespace rwls

#endif // NONE_H_
