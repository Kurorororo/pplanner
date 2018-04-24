#include "node/closed_list.h"

#include <cassert>

#include <iostream>
#include <vector>

#include "node/state_packer.h"
#include "node/node_vector.h"
#include "domain/state.h"

using namespace rwls;

int main() {
  NodeVector vec(0, 4);

  // 1, 4, 8, 31, 26, 9
  // [1, 4, 8], [31], [26], [9]
  std::vector<int> dom{2, 10, 100, 1 << 30, 1 << 25, 200};
  StatePacker packer(dom);

  ClosedList closed(2, 4);

  State state{0, 5, 33, 1 << 18, 1 << 12, 168};

  std::vector<uint32_t> packed0(4);
  packer.Pack(state, packed0.data());
  std::cout << "state0 hash=" << closed.Hash(packed0.data()) << std::endl;
  assert(!closed.Contain(vec, packed0.data()));
  int id = vec.GenerateNode(-1, -1, packed0.data());
  closed.Insert(vec, id);
  closed.Dump();
  assert(closed.Contain(vec, packed0.data()));

  state[2] = 9;
  std::vector<uint32_t> packed1(4);
  packer.Pack(state, packed1.data());
  std::cout << "state1 hash=" << closed.Hash(packed1.data()) << std::endl;
  assert(!closed.Contain(vec, packed1.data()));
  id = vec.GenerateNode(1, id, packed1.data());
  closed.Insert(vec, id);
  closed.Dump();
  assert(closed.Contain(vec, packed0.data()));
  assert(closed.Contain(vec, packed1.data()));

  state[3] = 51;
  std::vector<uint32_t> packed2(4);
  packer.Pack(state, packed2.data());
  std::cout << "state2 hash=" << closed.Hash(packed2.data()) << std::endl;
  assert(!closed.Contain(vec, packed2.data()));
  id = vec.GenerateNode(1, id, packed2.data());
  closed.Insert(vec, id);
  closed.Dump();
  assert(closed.Contain(vec, packed0.data()));
  assert(closed.Contain(vec, packed1.data()));
  assert(closed.Contain(vec, packed2.data()));

  state[4] = 129;
  std::vector<uint32_t> packed3(4);
  packer.Pack(state, packed3.data());
  std::cout << "state3 hash=" << closed.Hash(packed3.data()) << std::endl;
  assert(!closed.Contain(vec, packed3.data()));
  id = vec.GenerateNode(1, id, packed3.data());
  closed.Insert(vec, id);
  closed.Dump();
  assert(closed.Contain(vec, packed0.data()));
  assert(closed.Contain(vec, packed1.data()));
  assert(closed.Contain(vec, packed2.data()));
  assert(closed.Contain(vec, packed3.data()));

  std::cout << "passed" << std::endl;
}
