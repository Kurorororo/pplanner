#ifndef FILES_UTILS_H_
#define FILES_UTILS_H_

#include <queue>
#include <string>
#include <vector>

#include "sas_plus.h"

namespace pplanner {

std::queue<std::string> FileToLines(const std::string &filename);

void WritePlan(const SASPlus &sas, const std::vector<int> &plan);

} // namespace pplanner

#endif // FILES_UTILS_H_
