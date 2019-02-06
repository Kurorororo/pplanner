#ifndef WIDTH_H_
#define WIDTH_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include "evaluator.h"
#include "random_walk_evaluator.h"
#include "sas_plus.h"

namespace pplanner {

class Width : public Evaluator {
 public:
  Width() : is_ge_1_(false), problem_(nullptr) {}

  Width(std::shared_ptr<const SASPlus> problem, bool is_ge_1)
    : is_ge_1_(is_ge_1),
      tmp_state_(problem->n_variables()),
      tmp_facts_(problem->n_facts()),
      is_new_1_(problem->n_facts(), true),
      is_new_2_(problem->n_facts(), std::vector<bool>(problem->n_facts(), true)),
      problem_(problem) {}

  ~Width() {}

  int Evaluate(const std::vector<int> &state, int node) override;

  int Evaluate(const std::vector<int> &state, int node, int parent) override {
    return Evaluate(state, node);
  }

  int Evaluate(const std::vector<int> &state, int node,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override;

  int Evaluate(const std::vector<int> &state, int node, int parent,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return Evaluate(state, node, applicable, preferred);
  }

 private:
  bool is_ge_1_;
  std::vector<int> tmp_state_;
  std::vector<int> tmp_facts_;
  std::vector<bool> is_new_1_;
  std::vector<std::vector<bool> > is_new_2_;
  std::shared_ptr<const SASPlus> problem_;
};

class RWWidth : public RandomWalkEvaluator {
 public:
  RWWidth() : width_(nullptr) {
    width_ = std::unique_ptr<Width>(new Width());
  }

  RWWidth(std::shared_ptr<const SASPlus> problem, bool is_ge_1)
    : width_(nullptr) {
    width_ = std::unique_ptr<Width>(new Width(problem, is_ge_1));
  }

  ~RWWidth() {}

  int Evaluate(const std::vector<int> &state) override {
    return width_->Evaluate(state, -1);
  }

  int Evaluate(const std::vector<int> &state,
               const std::vector<int> &applicable,
               std::unordered_set<int> &preferred) override {
    return width_->Evaluate(state, -1, applicable, preferred);
  }

  void UpdateBest() override {}

  void LocalRestart() override {}

  void GlobalRestart() override {}

  void CopyBestToSearchGraph(int node,
                             std::shared_ptr<SearchGraph> graph) override {}

 private:
  std::unique_ptr<Width> width_;
};

} // namespace pplanner

#endif // WIDTH_H_
