#include "open_lists/open_list_impl_factory.h"

#include "gtest/gtest.h"

namespace pplanner {

TEST(OpenListImplFactoryTest, FIFO) {
  auto list = OpenListImplFactory("fifo");
  int node = 0;
  std::vector<int> values{0, 1, 2};
  list->Push(values, node);
  node = 1;
  values[0] = 1;
  list->Push(values, node);
  values[1] = 0;
  node = 2;
  list->Push(values, node);
  values[0] = 0;
  node = 3;
  list->Push(values, node);
  values[1] = 1;
  node = 4;
  list->Push(values, node);
  node = 5;
  list->Push(values, node);

  node = list->Pop();
  EXPECT_EQ(3, node);
  node = list->Pop();
  EXPECT_EQ(0, node);
  node = list->Pop();
  EXPECT_EQ(4, node);
  node = list->Pop();
  EXPECT_EQ(5, node);
  node = list->Pop();
  EXPECT_EQ(2, node);
  node = list->Pop();
  EXPECT_EQ(1, node);
}

} // namespace pplanner
