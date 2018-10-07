#include "dominance/lts.h"

#include <iostream>
#include <memory>
#include <string>

#include "sas_plus.h"

#include "utils/file_utils.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: sas_dtg <filename>" << std::endl;
    exit(1);
  }

  std::string filename = argv[1];
  auto lines = pplanner::FileToLines(filename);
  auto sas = std::make_shared<pplanner::SASPlus>();
  sas->InitFromLines(lines);
  auto ltss = pplanner::InitializeLTSs(sas);

  int i = 0;

  for (auto &lts: ltss) {
    std::cout << "var" << i << std::endl;
    lts->Dump();
    ++i;
  }
}
