#include "../preferred_gbfs.h"

#include <cassert>

#include <algorithm>
#include <limits>
#include <iostream>
#include <unordered_set>

#include "node/closed_list.h"
#include "domain/state.h"
#include "node/open_list.h"

using std::vector;

namespace rwls {

template<class H>
vector<int> PreferredGBFS<H>::operator()(Preferring *preferring,
                                         const Domain &domain, int n_boost,
                                         bool initial_boost) {
  TrieTable table = ConstructTable(domain);
  StatePacker packer(domain.dom);
  int block_size = packer.PackedSize();

  int size = NodeVectorSize(5000000000, block_size, 2);
  NodeVector vec(size, block_size);

  int goal = Search(preferring, domain, table, packer, vec, n_boost,
                    initial_boost);

  std::cout << "Preferred Operators Total: " << n_preferred_ << std::endl;

  return vec.ExtractPath(goal);
}

template<class H>
int PreferredGBFS<H>::Search(Preferring *preferring, const Domain &domain,
                             const TrieTable &table, const StatePacker &packer,
                             NodeVector &vec, int n_boost, bool initial_boost) {
  H heuristic;
  heuristic.Initialize(domain);
  AlternateOpenList<int, int> open(2);
  int block_size = packer.PackedSize();
  ClosedList closed(22, block_size);

  std::vector<int> applicable;
  std::unordered_set<int> preferred;

  vector<uint32_t> tmp_packed(block_size);
  packer.Pack(domain.initial, tmp_packed.data());

  int current_node = vec.GenerateNode(-1, -1, tmp_packed.data());
  ++generated_;
  int h = heuristic(domain.initial, domain);
  std::cout << "Initial heuristic value: " << h << std::endl;
  int best_seen = h;
  ++evaluated_;
  open.Push(0, h, current_node);

  State current(domain.initial);
  State child(current);

  if (initial_boost) open.Boost(1, n_boost);

  while (!open.IsEmpty()) {
    current_node = open.Pop();

    uint32_t *current_packed = vec.GetState(current_node);
    if (closed.Contain(vec, current_packed)) continue;
    closed.Insert(vec, current_node);

    packer.Unpack(current_packed, current);
    if (GoalCheck(domain.goal, current)) return current_node;
    FindFromTable(table, domain, current, applicable);

    if (applicable.empty()) {
      ++deadend_;
      continue;
    } else {
      ++expanded_;
    }

    preferring->extract(current, domain, applicable, preferred);

    for (auto o : applicable) {
      child = current;
      ApplyEffect(domain.effects[o], child);

      packer.Pack(child, tmp_packed.data());
      if (closed.Contain(vec, tmp_packed.data())) continue;

      int child_node = vec.GenerateNode(o, current_node, tmp_packed.data());
      ++generated_;

      int h = heuristic(child, domain);
      ++evaluated_;

      if (h == std::numeric_limits<int>::max()) {
        ++deadend_;
        continue;
      }

      if (h < best_seen) {
        best_seen = h;
        std::cout << "New best heuristic value: " << best_seen
                  << std::endl;
        std::cout << "[" << evaluated_ << " evaluated, "
                  << expanded_ << " expanded]" << std::endl;
        open.Boost(1, n_boost);
      }

      if (preferred.find(o) == preferred.end()) {
        open.Push(0, h, child_node);
      } else {
        open.Push(h, child_node);
        ++n_preferred_;
      }
    }
  }

  return -1;
}

} // namespace rwls
