#include "landmark/dtg.h"

#include <iostream>
#include <string>

#include "domain/domain.h"
#include "domain/parser.h"

using namespace rwls;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: sas_dtg <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  Domain domain;
  Parse(filename, &domain);
  auto dtgs = DTG::InitializeDTGs(domain);

  int i = 0;
  for (auto &dtg : dtgs) {
    std::cout << "var" << i << std::endl;
    dtg.Print();
    ++i;
  }
}
