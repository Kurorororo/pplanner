#include "utils/file_utils.h"

#include <fstream>

using std::queue;
using std::string;

namespace pplanner {

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
