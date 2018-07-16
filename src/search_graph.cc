#include "search_graph.h"

#include <algorithm>

namespace pplanner {

using std::vector;

bool SearchGraph::CloseIfNot(int node) {
  size_t block_size = packer_->block_size();
  auto packed = states_.data() + static_cast<size_t>(node) * block_size;
  size_t index = Find(HashValue(node), packed);

  if (closed_[index] != -1) return false;

  Close(index, node);

  return true;
}

int SearchGraph::GenerateNodeIfNotClosed(int action, int parent_node,
                                         uint32_t hash_value,
                                         const uint32_t *packed) {
  size_t index = Find(hash_value, packed);

  if (closed_[index] != -1) return -1;

  return GenerateNode(action, parent_node, hash_value, packed);
}

int SearchGraph::GenerateAndCloseNode(int action, int parent_node,
                                      uint32_t hash_value,
                                      const uint32_t *packed) {
  size_t index = Find(hash_value, packed);

  if (closed_[index] != -1) return -1;

  int node = GenerateNode(action, parent_node, hash_value, packed);
  Close(index, node);

  return node;
}

void SearchGraph::Close(size_t index, int node) {
  ++n_closed_;
  closed_[index] = node;

  if (2 * (n_closed_ + 1) > closed_.size())
    ResizeClosed();
}

void SearchGraph::ResizeClosed() {
  closed_exponent_ = 1;

  while ((1u << closed_exponent_) < 3 * n_closed_)
    ++closed_exponent_;

  closed_mask_ = (1u << closed_exponent_) - 1;
  std::vector<int> new_closed(1 << closed_exponent_, -1);

  for (int k=0, m=closed_.size(); k<m; ++k) {
    int id = closed_[k];

    if (id != -1) {
      size_t i = hash_values_[id] & closed_mask_;

      while (new_closed[i] != -1)
        i = i == (new_closed.size() - 1) ? 0 : i + 1;

      new_closed[i] = id;
    }
  }

  closed_.swap(new_closed);
}

size_t SearchGraph::Find(uint32_t hash_value, const uint32_t *packed) const {
  size_t i = hash_value & closed_mask_;

  if (closed_[i] == -1) return i;

  size_t b_size = packer_->block_size();

  while (closed_[i] != -1) {
    auto found = states_.data() + static_cast<size_t>(closed_[i]) * b_size;
    if (BytesEqual(b_size, packed, found)) break;
    i= (i== closed_.size() - 1) ? 0 : i + 1;
  }

  return i;
}

vector<int> ExtractPath(std::shared_ptr<const SearchGraph> graph, int node) {
  if (node == -1) return std::vector<int>{-1};

  vector<int> result;

  while (graph->Parent(node) != -1) {
    result.push_back(graph->Action(node));
    node = graph->Parent(node);
  }

  std::reverse(result.begin(), result.end());

  return result;
}

} // namespace pplanner
