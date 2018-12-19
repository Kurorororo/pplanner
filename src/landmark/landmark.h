#ifndef LANDMARK_H_
#define LANDMARK_H_

#include <algorithm>
#include <utility>
#include <vector>

#include "hash/pair_hash.h"

namespace pplanner {

class Landmark {
 public:
  Landmark() {}

  explicit Landmark(const std::vector<std::pair<int, int> > &var_values)
    : var_values_(var_values) {
    std::sort(var_values_.begin(), var_values_.end());
  }

  explicit Landmark(const std::pair<int, int> &p) {
    var_values_.push_back(p);
  }

  Landmark(int var, int value) {
    var_values_.push_back(std::make_pair(var, value));
  }

  ~Landmark() {}

  bool operator==(const Landmark &l) const {
    return var_values_ == l.var_values_;
  }

  bool operator!=(const Landmark &l) const {
    return var_values_ != l.var_values_;
  }

  int size() const { return var_values_.size(); }

  bool IsEmpty() const { return size() == 0; }

  bool IsFact() const { return size() == 1; }

  void Clear() { var_values_.clear(); }

  void Add(const std::pair<int, int> &p) {
    auto result = std::find(var_values_.begin(), var_values_.end(), p);
    if (result != var_values_.end()) return;
    var_values_.push_back(p);
    std::sort(var_values_.begin(), var_values_.end());
  }

  int Var(int i) const { return var_values_[i].first; }

  int Value(int i) const { return var_values_[i].second; }

  std::pair<int, int> VarValue(int i) const { return var_values_[i]; }

  std::size_t Hash() const;

  bool IsImplicated(const std::vector<int> &state) const;

  bool IsImplicated(const std::pair<int, int> &p) const;

  bool Overlap(const Landmark &l) const;

  void Dump() const;

 private:
  std::vector<std::pair<int, int> > var_values_;
};

struct LandmarkHash {
  std::size_t operator()(const Landmark &landmark) const {
    return landmark.Hash();
  }
};

} // namespace pplanner

#endif // LANDMARK_H_
