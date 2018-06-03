#include "sas_plus.h"

#include <iostream>
#include <string>

#include "utils/file_utils.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: sas_parser <filename>" << std::endl;
    exit(1);
  }

  std::string filename = argv[1];
  auto lines = pplanner::FileToLines(filename);
  pplanner::SASPlus sas;
  sas.InitFromLines(lines);
  sas.Dump();
}
