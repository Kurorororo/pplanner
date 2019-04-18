#ifndef BMRW_H_
#define BMRW_H_

#include <array>
#include <memory>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "landmark/landmark_count_base.h"
#include "open_list.h"
#include "sas_plus.h"
#include "search.h"
#include "search_graph/search_graph_with_landmarks.h"
#include "successor_generator.h"

namespace pplanner {

struct Batch {
  std::vector<int> parents;
  std::vector<int> hs;
  std::vector<std::vector<int> > states;
  std::vector<std::vector<uint8_t> > landmarks;
  std::vector<std::vector<int> > sequences;

  Batch(std::size_t size, std::size_t n_variables, std::size_t n_landmark_bytes)
    : parents(size),
      hs(size),
      states(size, std::vector<int>(n_variables)),
      landmarks(size, std::vector<uint8_t>(n_landmark_bytes, 0)),
      sequences(size) {}
};

class BMRW : public Search {
 public:
  BMRW(std::shared_ptr<const SASPlus> problem,
       const boost::property_tree::ptree &pt)
      : use_preferred_(false),
        n_batch_(5120),
        n_elite_(100),
        walk_length_(10),
        generated_(0),
        expanded_(0),
        evaluated_(0),
        dead_ends_(0),
        problem_(problem),
        generator_(std::make_shared<SuccessorGenerator>(problem)),
        graph_(nullptr),
        lmcount_(nullptr),
        open_list_(nullptr) {
    Init(pt);
  }

  ~BMRW() {}

  std::vector<int> Plan() override {
    int goal = Search();

    return ExtractPlan(goal);
  }

  void DumpStatistics() const override;

 private:
  void Init(const boost::property_tree::ptree &pt);

  void InitialEvaluate();

  void PopStates(Batch &batch);

  void RandomWalk(Batch &batch);

  void GenerateChildren(int parent, int h, const std::vector<int> &state);

  int PushStates(const Batch &batch);

  void Restart();

  int Search();

  std::vector<int> ExtractPlan(int node);

  bool use_preferred_;
  int n_batch_;
  int n_elite_;
  int walk_length_;
  int generated_;
  int expanded_;
  int evaluated_;
  int dead_ends_;
  int best_h_;
  std::vector<std::vector<int> > sequences_;
  std::shared_ptr<const SASPlus> problem_;
  std::shared_ptr<SuccessorGenerator> generator_;
  std::shared_ptr<SearchGraphWithLandmarks> graph_;
  std::shared_ptr<LandmarkCountBase> lmcount_;
  std::unique_ptr<OpenList<int, int> > open_list_;
};

}  // namespace pplanner

#endif  // BMRW_H_