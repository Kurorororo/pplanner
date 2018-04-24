#include "gbfs_lmc.h"

#include <cassert>

#include <algorithm>
#include <limits>
#include <iostream>
#include <unordered_set>

#include "node/closed_list.h"
#include "node/open_list.h"
#include "domain/state.h"

using std::vector;

namespace rwls {

int NodeVectorSizeLmc(size_t block_size, size_t ac_bytes) {
  size_t size = ((block_size + 2) * sizeof(int) + ac_bytes * sizeof(uint8_t));

  return static_cast<int>(5000000000 / size);
}

vector<int> GBFSLmc::operator()(const Domain &domain) {
  TrieTable table = ConstructTable(domain);
  StatePacker packer(domain.dom);
  int block_size = packer.PackedSize();

  LandmarkCount lmc;
  lmc.Initialize(domain);
  size_t ac_bytes = lmc.accepted_bytes;

  int size = NodeVectorSizeLmc(block_size, ac_bytes);
  NodeVector vec(size, block_size);
  size_t lms_size = static_cast<size_t>(size) * ac_bytes;
  vector<uint8_t> lm_vec;
  lm_vec.reserve(lms_size);

  int goal = Search(domain, table, packer, lmc, vec, lm_vec);

  return vec.ExtractPath(goal);
}

int GBFSLmc::Search(const Domain &domain, const TrieTable &table,
                    const StatePacker &packer,LandmarkCount &lmc,
                    NodeVector &vec, vector<uint8_t> &lm_vec) {
  OpenList<int, int> open;
  int block_size = packer.PackedSize();
  ClosedList closed(22, block_size);
  size_t ac_bytes = lmc.accepted_bytes;

  int best_seen = std::numeric_limits<int>::max();
  std::vector<int> applicable;

  vector<uint32_t> tmp_packed(block_size);
  packer.Pack(domain.initial, tmp_packed.data());

  int current_node = vec.GenerateNode(-1, -1, tmp_packed.data());
  vec.GenerateBits(ac_bytes, lm_vec);
  uint8_t *accepted = lm_vec.data();
  ++generated_;
  int h = lmc(domain.initial, domain, nullptr, accepted);
  std::cout << "Initial heuristic value for lmcount: " << h
            << std::endl;
  ++evaluated_;
  open.Push(h, current_node);

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
      ++deadend_;
      continue;
    } else {
      ++expanded_;
    }

    int h_min = std::numeric_limits<int>::max();
    const uint8_t *parent_accepted = vec.GetBits(ac_bytes, current_node, lm_vec);

    for (auto o : applicable) {
      child = current;
      ApplyEffect(domain.effects[o], child);

      packer.Pack(child, tmp_packed.data());
      if (closed.Contain(vec, tmp_packed.data())) continue;

      int child_node = vec.GenerateNode(o, current_node, tmp_packed.data());
      vec.GenerateBits(ac_bytes, lm_vec);
      uint8_t *accepted = vec.GetBits(ac_bytes, child_node, lm_vec);
      ++generated_;

      int h = lmc(child, domain, parent_accepted, accepted);
      ++evaluated_;

      if (h == std::numeric_limits<int>::max()) {
        ++deadend_;
        continue;
      }

      if (h < h_min) h_min = h;
      open.Push(h, child_node);
    }

    if (h_min < best_seen) {
      best_seen = h_min;
      std::cout << "New best heuristic value for lmcount: " << best_seen
                << std::endl;
      std::cout << "[" << evaluated_ << " evaluated, "
                << expanded_ << " expanded]" << std::endl;
    }
  }

  return -1;
}

} // namespace rwls
