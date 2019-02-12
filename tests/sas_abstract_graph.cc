#include "multithread_search/abstract_graph.h"

#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

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

  auto graph_by_dtg = ConstructByDTG(sas, 50000);
  graph_by_dtg->BuildInferenceScope();
  std::cout << "graph constructed by dtg" << std::endl;
  graph_by_dtg->Dump();

  std::vector<int> candidates(sas->n_variables());
  std::iota(candidates.begin(), candidates.end(), 0);
  std::vector<int> selected;
  std::shared_ptr<pplanner::AbstractGraph> best_graph = nullptr;

  std::cout << "constructing graph by greedy algorithm" << std::endl;

  while (true) {
    auto graph = GreedySelect(sas, candidates, selected, 50000);

    if (graph == nullptr) break;

    best_graph = graph;
    selected = graph->vars();

    std::cout << "selected:";

    for (auto v : selected)
      std::cout << v << " ";

    std::cout << std::endl;
    std::cout << "#nodes=" << graph->n_nodes() << std::endl;
    std::cout << "#successors=" << graph->max_out_degree() << std::endl;
    std::cout << "delta=" << graph->Delta() << std::endl;
  }

  if (best_graph == nullptr) {
    std::cout << "cannot find abstract graph" << std::endl;
  } else {
    best_graph->BuildInferenceScope();
    best_graph->Dump();
  }
}
