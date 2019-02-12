#ifndef LEXICAL_ORDER_H_
#define LEXICAL_ORDER_H_

#include <vector>

namespace pplanner {

int LexicalOrder(const std::vector<int> &values,
                 const std::vector<int> &ranges);

std::vector<int> OrderToValues(int order, const std::vector<int> &ranges);

}

#endif // LEXICAL_ORDER_H_
