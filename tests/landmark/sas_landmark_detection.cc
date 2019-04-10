#include "landmark/landmark_detection.h"

#include <iostream>
#include <string>

#include "heuristics/relaxed_sas_plus.h"
#include "landmark/generating_orderings.h"
#include "landmark/landmark_graph.h"
#include "sas_plus.h"
#include "utils/file_utils.h"

using namespace pplanner;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: sas_landmark_detection <filename>" << std::endl;
    exit(1);
  }

  std::string filename = argv[1];
  auto lines = FileToLines(filename);
  auto sas = std::make_shared<pplanner::SASPlus>();
  sas->InitFromLines(lines);
  auto r_sas = std::make_shared<RelaxedSASPlus>(sas, false);
  auto graph = std::make_shared<LandmarkGraph>(sas);

  IdentifyLandmarks(sas, r_sas, graph);
  AddOrderings(sas, r_sas, graph);
  HandleCycles(graph);

  std::cout << "landmarks" << std::endl;

  auto landmarks = graph->CopyLandmarks();
  auto remove_cond = [](const Landmark &l) { return l.IsEmpty(); };
  auto end = std::remove_if(landmarks.begin(), landmarks.end(), remove_cond);
  landmarks.erase(end, landmarks.end());
  auto lambda = [](const Landmark &l1, const Landmark &l2) {
    if (l1.size() == l2.size()) return l1.VarValue(0) > l2.VarValue(0);
    return l1.size() > l2.size();
  };
  std::sort(landmarks.begin(), landmarks.end(), lambda);

  for (auto &landmark : landmarks) {
    int id = graph->ToId(landmark);
    std::cout << "id: " << id << " ";
    landmark.Dump();
    std::cout << "Achievers (" << graph->GetPossibleAchieversSize(id) << ", "
              << graph->GetFirstAchieversSize(id) << ")" << std::endl;

    for (auto init_id : graph->GetInitIdsByTermId(id)) {
      std::cout << "    <-_";
      if (graph->GetOrderingType(init_id, id) == LandmarkGraph::GREEDY)
        std::cout << "gn ";
      if (graph->GetOrderingType(init_id, id) == LandmarkGraph::NATURAL)
        std::cout << "nat ";
      if (graph->GetOrderingType(init_id, id) == LandmarkGraph::REASONABLE)
        std::cout << "r ";
      if (graph->GetOrderingType(init_id, id) == LandmarkGraph::OBEDIENT)
        std::cout << "or ";

      std::cout << "id:" << init_id << " ";
      auto init = graph->GetLandmark(init_id);
      init.Dump();
      std::cout << std::endl;
    }

    for (auto term_id : graph->GetTermIdsByInitId(id)) {
      std::cout << "    ->_";
      if (graph->GetOrderingType(id, term_id) == LandmarkGraph::GREEDY)
        std::cout << "gn ";
      if (graph->GetOrderingType(id, term_id) == LandmarkGraph::NATURAL)
        std::cout << "nat ";
      if (graph->GetOrderingType(id, term_id) == LandmarkGraph::REASONABLE)
        std::cout << "r ";
      if (graph->GetOrderingType(id, term_id) == LandmarkGraph::OBEDIENT)
        std::cout << "o_r ";
      std::cout << "id:" << term_id << " ";
      auto term = graph->GetLandmark(term_id);
      term.Dump();
      std::cout << std::endl;
    }

    std::cout << std::endl;
  }
  std::cout << std::endl;
}
