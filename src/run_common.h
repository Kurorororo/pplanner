#ifndef RUN_COMMON_H_
#define RUN_COMMON_H_

#include <vector>

#include "domain/domain.h"

namespace rwls {

struct SearchStatistics {
  double search_time;
  int generated;
  int expanded;
  int evaluated;
  int deadend;
};

void WritePlan(const Domain &domain, const std::vector<int> &plan);

void PrintStatistics(const SearchStatistics &stat);

template<class T>
void SetStatistics(const T &solver, SearchStatistics &stat) {
  stat.generated = solver.generated();
  stat.expanded = solver.expanded();
  stat.evaluated = solver.evaluated();
  stat.deadend = solver.deadend();
}

}

#endif // RUN_COMMON_H_
