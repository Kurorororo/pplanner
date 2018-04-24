#include "node/node_vector.h"

#include <cassert>

#include <iostream>
#include <vector>

#include "node/state_packer.h"

using namespace rwls;

int main() {
  NodeVector vec(0, 4);

  std::vector<uint32_t> packed0{3, 2, 1, 5};
  int id = vec.GenerateNode(-1, -1, packed0.data());
  assert(0 == id);
  assert(-1 == vec.GetAction(id));
  assert(-1 == vec.GetParent(id));
  assert(BytesEqual(4, packed0.data(), vec.GetState(id)));

  std::vector<uint32_t> packed1{3, 3, 0, 5};
  id = vec.GenerateNode(1, id, packed1.data());
  assert(1 == id);
  assert(1 == vec.GetAction(id));
  assert(0 == vec.GetParent(id));
  assert(BytesEqual(4, packed1.data(), vec.GetState(id)));
  assert(-1 == vec.GetAction(0));
  assert(-1 == vec.GetParent(0));
  assert(BytesEqual(4, packed0.data(), vec.GetState(0)));

  std::vector<int> another;
  vec.InsertToAnother(2, another);
  assert(2 == another[id]);

  std::cout << "passed" << std::endl;
}
