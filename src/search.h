#ifndef SEARCH_H_
#define SEARCH_H_

#include <vector>

namespace pplanner {

class Search {
 public:
  virtual ~Search() = 0;

  virtual std::vector<int> Plan() = 0;

  virtual void DumpStatistics() const = 0;
};

} // namespace pplanner

#endif // SEARCH_H_
