#include "utils/file_utils.h"

namespace pplanner {

using std::queue;
using std::string;

queue<string> FileToLines(const string &filename) {
  std::ifstream input;
  input.open(filename, std::ios::in);
  std::string buffer;
  std::queue<std::string> lines;

  while (std::getline(input, buffer))
    lines.push(buffer);

  input.close();

  return lines;
}

} // namespace pplanner
