#include "search_graph/distributed_search_graph.h"

#include <iostream>

namespace pplanner {

using std::vector;

int DistributedSearchGraph::GenerateNodeIfNotClosed(int action, int parent_node,
                                                    uint32_t hash_value,
                                                    const uint32_t *packed,
                                                    int parent_rank) {
  int node = GenerateNodeIfNotClosed(action, parent_node, hash_value, packed);

  if (node != -1) AddMoreProperties(parent_rank);

  return node;
}

int DistributedSearchGraph::GenerateNodeIfNotClosed(int action, int parent_node,
                                                    const vector<int> &state,
                                                    int parent_rank) {
  int node = GenerateNodeIfNotClosed(action, parent_node, state);

  if (node != -1) AddMoreProperties(parent_rank);

  return node;
}

int DistributedSearchGraph::GenerateNodeIfNotClosed(int action, int parent_node,
                                                    const vector<int> &parent,
                                                    const vector<int> &state,
                                                    int parent_rank) {
  int node = GenerateNodeIfNotClosed(action, parent_node, parent, state);

  if (node != -1) AddMoreProperties(parent_rank);

  return node;
}

int DistributedSearchGraph::GenerateNodeIfNotClosed(const unsigned char *d) {
  int info[3];
  memcpy(info, d, 3 * sizeof(int));
  uint32_t hash_value;
  memcpy(&hash_value, d + 3 * sizeof(int), sizeof(uint32_t));
  const uint32_t *packed = reinterpret_cast<const uint32_t*>(
      d + 3 * sizeof(int) + sizeof(uint32_t));

  return GenerateNodeIfNotClosed(info[0], info[1], hash_value, packed, info[2]);
}

int DistributedSearchGraph::GenerateNode(const unsigned char *d, int *h) {
  int info[4];
  memcpy(info, d, 4 * sizeof(int));
  *h = info[0];
  uint32_t hash_value;
  memcpy(&hash_value, d + 3 * sizeof(int), sizeof(uint32_t));
  const uint32_t *packed = reinterpret_cast<const uint32_t*>(
      d + 4 * sizeof(int) + sizeof(uint32_t));
  return GenerateNode(info[1], info[2], hash_value, packed, info[3]);
}

void DistributedSearchGraph::BufferNode(int action, int parent_node,
                                        const vector<int> &parent,
                                        const vector<int> &state,
                                        unsigned char *buffer) {
  int info[3];
  info[0] = action;
  info[1] = parent_node;
  info[2] = rank_;
  memcpy(buffer, info, 3 * sizeof(int));
  uint32_t hash_value = HashByDifference(action, parent_node, parent, state);
  memcpy(buffer + 3 * sizeof(int), &hash_value, sizeof(uint32_t));
  uint32_t *packed = reinterpret_cast<uint32_t*>(
      buffer + 3 * sizeof(int) + sizeof(uint32_t));
  Pack(state, packed);
}

} // namespace pplanner
