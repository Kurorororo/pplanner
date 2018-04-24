#include "landmark/landmark_detection.h"

#include <iostream>
#include <string>

#include "domain/domain.h"
#include "domain/parser.h"
#include "landmark/generating_orderings.h"
#include "landmark/landmark_graph.h"
#include "heuristic/graphplan.h"

using namespace rwls;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: sas_landmark_detection <filename>" << std::endl;
    exit(1);
  }
  std::string filename = argv[1];
  Domain domain;
  Parse(filename, &domain);
  GraphSchema schema;
  InitializeSchema(domain, &schema);
  LandmarkGraph(graph);

  IdentifyLandmarks(domain, schema, &graph);
  AddOrderings(domain, schema, &graph);
  HandleCycles(&graph);

  std::cout << "landmarks" << std::endl;

  auto landmarks = graph.CopyLandmarks();
  auto remove_cond = [](const Landmark &l) { return l.IsEmpty(); };
  auto end = std::remove_if(landmarks.begin(), landmarks.end(), remove_cond);
  landmarks.erase(end, landmarks.end());
  auto lambda = [](const Landmark &l1, const Landmark &l2) {
    if (l1.GetSize() == l2.GetSize())
      return l1.GetVarValue() > l2.GetVarValue();
    return l1.GetSize() > l2.GetSize();
  };
  std::sort(landmarks.begin(), landmarks.end(), lambda);

  for (auto &landmark : landmarks) {
    size_t id = graph.ToId(landmark);
    std::cout << "id: " << id << " ";
    landmark.Print();
    if (landmark.IsImplicated(domain.goal)) std::cout << "(goal) ";
    std::cout << "Achievers (" << graph.GetPossibleAchieversSize(id)
              << ", " << graph.GetFirstAchieversSize(id) << ")" << std::endl;
    for (auto init_id : graph.GetInitIdsByTermId(id)) {
      std::cout << "    <-_";
      if (graph.GetOrderingType(init_id, id) == LandmarkGraph::GREEDY)
        std::cout << "gn ";
      if (graph.GetOrderingType(init_id, id) == LandmarkGraph::NATURAL)
        std::cout << "nat ";
      if (graph.GetOrderingType(init_id, id) == LandmarkGraph::REASONABLE)
        std::cout << "r ";
      if (graph.GetOrderingType(init_id, id) == LandmarkGraph::OBEDIENT)
        std::cout << "or ";
      std::cout << "id:" << init_id << " ";
      auto init = graph.GetLandmark(init_id);
      init.Print();
      if (init.IsImplicated(domain.goal)) std::cout << "(goal)";
      std::cout << std::endl;
    }
    for (auto term_id : graph.GetTermIdsByInitId(id)) {
      std::cout << "    ->_";
      if (graph.GetOrderingType(id, term_id) == LandmarkGraph::GREEDY)
        std::cout << "gn ";
      if (graph.GetOrderingType(id, term_id) == LandmarkGraph::NATURAL)
        std::cout << "nat ";
      if (graph.GetOrderingType(id, term_id) == LandmarkGraph::REASONABLE)
        std::cout << "r ";
      if (graph.GetOrderingType(id, term_id) == LandmarkGraph::OBEDIENT)
        std::cout << "o_r ";
      std::cout << "id:" << term_id << " ";
      auto term = graph.GetLandmark(term_id);
      term.Print();
      if (term.IsImplicated(domain.goal)) std::cout << "(goal)";
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}
