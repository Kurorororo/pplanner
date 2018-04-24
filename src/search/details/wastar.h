#include "../wastar.h"

#include <cassert>

#include <algorithm>
#include <limits>
#include <iostream>
#include <utility>

#include "node/closed_list.h"
#include "node/open_list.h"
#include "domain/state.h"

using std::vector;

namespace rwls {

int NodeVectorSize(int block_size) {
  return 5000000000  / ((block_size + 3) * sizeof(int));
}

template<class H>
vector<int> WAstar<H>::operator()(int weight, const Domain &domain) {
  TrieTable table = ConstructTable(domain);
  StatePacker packer(domain.dom);
  int block_size = packer.PackedSize();

  int size = NodeVectorSize(block_size);
  NodeVector vec(size, block_size);
  vector<int> g_values(size, -1);

  int goal = Search(weight, domain, table, packer, vec, g_values);

  return ExtractPath(vec, goal);
}

template<class H>
int WAstar<H>::Search(int weight, const Domain &domain, const TrieTable &table,
                       const StatePacker &packer, NodeVector &vec,
                       vector<int> &g_values) {
  H heuristic;
  heuristic.Initialize(domain);
  OpenList<std::pair<int, int>, int> open;
  int block_size = packer.PackedSize();
  ClosedList closed(22, block_size);
  int size = NodeVectorSize(block_size);

  int best_seen = std::numeric_limits<int>::max();
  std::vector<int> applicable;

  vector<uint32_t> tmp_packed(block_size);
  packer.Pack(domain.initial, tmp_packed.data());

  int current_node = vec.GenerateNode(-1, -1, tmp_packed.data());
  vec.InsertToAnother(0, g_values);
  ++generated;
  int h = heuristic(domain.initial, domain);
  ++evaluated;
  open.Push(std::make_pair(weight * h, 0), current_node);

  State current(domain.initial);
  State child(current);

  while (!open.IsEmpty()) {
    current_node = open.Pop();

    uint32_t *current_packed = vec.GetState(current_node);
    if (closed.Contain(vec, current_packed)) continue;
    closed.Insert(vec, current_node);

    packer.Unpack(current_packed, current);
    if (GoalCheck(domain.goal, current)) return current_node;
    FindFromTable(table, domain, current, applicable);

    if (applicable.empty()) {
      ++deadend;
      continue;
    } else {
      ++expanded;
    }

    int h_min = std::numeric_limits<int>::max();
    int parent_g = g_values[current_node];

    for (auto o : applicable) {
      child = current;
      ApplyEffect(domain.effects[o], child);

      packer.Pack(child, tmp_packed.data());
      if (closed.Contain(vec, tmp_packed.data())) continue;

      int child_node = vec.GenerateNode(o, current_node, tmp_packed.data());
      int g = parent_g + domain.costs[o];
      vec.InsertToAnother(g, g_values);
      ++generated;

      int h = heuristic(child, domain);
      ++evaluated;

      if (h == std::numeric_limits<int>::max()) {
        ++deadend;
        continue;
      }

      if (h < h_min) h_min = h;
      int f = weight * h + g;
      open.Push(std::make_pair(f, h), child_node);
    }

    if (h_min < best_seen) {
      best_seen = h_min;
      std::cout << "New best heuristic value: " << best_seen
                << std::endl;
      std::cout << "[" << evaluated << " evaluated, "
                << expanded << " expanded]" << std::endl;
    }
  }

  return -1;
}

template<class H>
vector<int> WAstar<H>::ExtractPath(const NodeVector &vec, int node) {
  if (node == -1) return std::vector<int>{-1};
  std::vector<int> result;

  while (vec.GetParent(node) != -1) {
    result.push_back(vec.GetAction(node));
    node = vec.GetParent(node);
  }

  std::reverse(result.begin(), result.end());

  return result;
}

} // namespace rwls
