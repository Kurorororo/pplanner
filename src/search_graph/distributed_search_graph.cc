#include "search_graph/distributed_search_graph.h"

namespace pplanner {

using std::vector;

void DistributedSearchGraph::BufferNode(int to_rank, int parent, int action,
                                        int parent_rank,
                                        const vector<int> &state,
                                        unsigned char *buffer) {
  int info[3];
  info[0] = parent;
  info[1] = action;
  info[2] = parent_rank;
  memcpy(buffer, info, 3 * sizeof(int));
  uint32_t *packed = reinterpret_cast<uint32_t*>(buffer + 3 * sizeof(int));
  PackState(state, packed);
}

int DistributedSearchGraph::GenerateNodeIfNotClosed(const unsigned char *d) {
  int info[3];
  memcpy(info, d, 3 * sizeof(int));
  const uint32_t *packed = reinterpret_cast<const uint32_t*>(
      d + 3 * sizeof(int));
  int node = GenerateNodeIfNotClosed(packed, info[0], info[1], false, info[2]);

  return node;
}

int DistributedSearchGraph::GenerateNode(const unsigned char *d, int *h) {
  int info[4];
  memcpy(info, d, 4 * sizeof(int));
  const uint32_t *packed = reinterpret_cast<const uint32_t*>(
      d + 4 * sizeof(int));
  int node = GenerateNode(packed, info[1], info[2], false, info[3]);
  *h = info[0];

  return node;
}

} // namespace pplanner
