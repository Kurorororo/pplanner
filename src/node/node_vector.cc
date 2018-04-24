#include "node/node_vector.h"

#include <algorithm>

namespace rwls {

int NodeVectorSize(size_t ram_size, size_t block_size, size_t additional) {
  size_t size = ram_size / ((block_size + additional) * sizeof(int));

  return static_cast<int>(size);
}

std::vector<int> NodeVector::ExtractPath(int node) {
  if (node == -1) return std::vector<int>{-1};
  std::vector<int> result;

  while (GetParent(node) != -1) {
    result.push_back(GetAction(node));
    node = GetParent(node);
  }

  std::reverse(result.begin(), result.end());

  return result;
}

} // namespace rwls
