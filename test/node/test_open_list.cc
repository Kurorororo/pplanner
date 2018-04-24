#include "node/open_list.h"

#include <cassert>

#include <iostream>
#include <utility>

using namespace rwls;

void TestIsEmpty() {
  OpenList<int, int> open_list;

  assert(true == open_list.IsEmpty());

  std::cout << "passed IsEmpty" << std::endl;
}

void TestPush() {
  OpenList<int, int> open_list;

  open_list.Push(0, 0);
  assert(false == open_list.IsEmpty());

  std::cout << "passed Push" << std::endl;
}

void TestTop() {
  OpenList<int, int> open_list;

  open_list.Push(1, 10);
  assert(10 == open_list.Top());
  open_list.Push(2, 5);
  assert(10 == open_list.Top());
  open_list.Push(0, 3);
  assert(3 == open_list.Top());

  std::cout << "passed Top" << std::endl;
}

void TestPop() {
  OpenList<int, int> open_list;

  open_list.Push(0, 10);
  auto m = open_list.Pop();
  assert(true == open_list.IsEmpty());
  assert(10 == m);

  open_list.Push(1, 1);
  open_list.Push(2, 2);
  open_list.Push(3, 3);
  open_list.Push(0, 4);

  m = open_list.Pop();
  assert(4 == m);
  m = open_list.Pop();
  assert(1 == m);
  m = open_list.Pop();
  assert(2 == m);
  m = open_list.Pop();
  assert(3 == m);
  assert(true == open_list.IsEmpty());

  std::cout << "passed Pop" << std::endl;
}

void TestPairIsEmpty() {
  OpenList<std::pair<int, int>, int> open_list;

  assert(true == open_list.IsEmpty());

  std::cout << "passed IsEmpty" << std::endl;
}

void TestPairPush() {
  OpenList<std::pair<int, int>, int> open_list;

  open_list.Push(std::make_pair(0, 0), 0);
  assert(false == open_list.IsEmpty());

  std::cout << "passed pair Push" << std::endl;
}

void TestPairTop() {
  OpenList<std::pair<int, int>, int> open_list;

  open_list.Push(std::make_pair(1, 3), 10);
  assert(10 == open_list.Top());
  open_list.Push(std::make_pair(2, 1), 5);
  assert(10 == open_list.Top());
  open_list.Push(std::make_pair(1, 2), 3);
  assert(3 == open_list.Top());
  open_list.Push(std::make_pair(0, 5), 2);
  assert(2 == open_list.Top());

  std::cout << "passed pair Top" << std::endl;
}

void TestPairPop() {
  OpenList<std::pair<int, int>, int> open_list;

  open_list.Push(std::make_pair(0, 2), 10);
  auto m = open_list.Pop();
  assert(true == open_list.IsEmpty());
  assert(10 == m);

  open_list.Push(std::make_pair(1, 1), 1);
  open_list.Push(std::make_pair(1, 2), 2);
  open_list.Push(std::make_pair(3, 0), 3);
  open_list.Push(std::make_pair(0, 2), 4);

  m = open_list.Pop();
  assert(4 == m);
  m = open_list.Pop();
  assert(1 == m);
  m = open_list.Pop();
  assert(2 == m);
  m = open_list.Pop();
  assert(3 == m);
  assert(true == open_list.IsEmpty());

  std::cout << "passed pair Pop" << std::endl;
}

int main() {
  TestIsEmpty();
  TestPush();
  TestTop();
  TestPop();
  TestPairIsEmpty();
  TestPairPush();
  TestPairTop();
  TestPairPop();
}
