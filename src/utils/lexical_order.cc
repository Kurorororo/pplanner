#include "utils/lexical_order.h"

namespace pplanner {

int LexicalOrder(const std::vector<int> &values,
                 const std::vector<int> &ranges) {
  int order = 0;

  for (int i = 0, n = values.size(); i < n; ++i) {
    int product = values[i];

    for (int j = i + 1; j < n; ++j)
      product *= ranges[j];

    order += product;
  }

  return order;
}

std::vector<int> OrderToValues(int order, const std::vector<int> &ranges) {
  std::vector<int> values(ranges.size());

  for (int i = 0, n = ranges.size(); i < n; ++i) {
    int product = 1;

    for (int j = i + 1; j < n; ++j)
      product *= ranges[j];

    values[i] = order / product;
    order = order % product;
  }

  return values;
}

} // namespace pplanner
