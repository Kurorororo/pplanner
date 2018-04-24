#include "node/state_packer.h"

#include <cassert>

#include <iostream>
#include <vector>

#include "domain/state.h"

using namespace rwls;

void TestPackUnpack() {
  // 1, 4, 8, 31, 26, 9
  // [1, 4, 8], [31], [26], [9]
  std::vector<int> dom{2, 10, 100, 1 << 30, 1 << 25, 200};
  StatePacker packer(dom);

  assert(4 == packer.PackedSize());

  State state{0, 5, 33, 1 << 18, 1 << 12, 168};
  std::vector<uint32_t> packed(4);
  packer.Pack(state, packed.data());
  State state2(6);
  packer.Unpack(packed.data(), state2);

  assert(state == state2);

  std::cout << "passed Pack UnPack" << std::endl;
}

void TestBytesEqual() {
  // 1, 4, 8, 31, 26, 9
  // [1, 4, 8], [31], [26], [9]
  std::vector<int> dom{2, 10, 100, 1 << 30, 1 << 25, 200};
  StatePacker packer(dom);

  assert(4 == packer.PackedSize());

  State state{0, 5, 33, 1 << 18, 1 << 12, 168};
  std::vector<uint32_t> packed(4);
  packer.Pack(state, packed.data());
  std::vector<uint32_t> packed2(4);
  packer.Pack(state, packed2.data());

  assert(BytesEqual(4, packed.data(), packed2.data()));

  state[2] = 3;
  packer.Pack(state, packed2.data());

  assert(!BytesEqual(4, packed.data(), packed2.data()));

  std::cout << "passed BytesEqual" << std::endl;
}

int main() {
  TestPackUnpack();
  TestBytesEqual();
}
