#ifndef FILES_UTILS_H_
#define FILES_UTILS_H_

#include <queue>
#include <string>

namespace pplanner {

std::queue<std::string> FileToLines(const std::string &filename);

} // namespace pplanner

#endif // FILES_UTILS_H_
