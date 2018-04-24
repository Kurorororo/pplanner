#ifndef LANDMARK_H_
#define LANDMARK_H_

#include <algorithm>
#include <vector>

#include "domain/state.h"
#include "domain/var_value.h"

namespace rwls {

class Landmark {
 public:
  Landmark() {}

  Landmark(const Landmark &landmark) {
    var_values_ = landmark.var_values_;
  }

  explicit Landmark(VarValue var_value) {
    var_values_.push_back(var_value);
  }

  explicit Landmark(Landmark &landmark) {
    var_values_ = landmark.var_values_;
  }

  Landmark(int var, int value) {
    VarValue var_value;
    EncodeVarValue(var, value, &var_value);
    var_values_.push_back(var_value);
  }

  ~Landmark() {}

  inline Landmark &operator=(const Landmark &landmark) {
    var_values_ = landmark.GetVarValues();
    return *this;
  }

  inline bool operator==(const Landmark &landmark) const {
    return var_values_ == landmark.var_values_;
  };

  inline bool operator!=(const Landmark &landmark) const {
    return var_values_ != landmark.var_values_;
  };

  inline bool IsEmpty() const {
    return var_values_.empty();
  }

  inline bool IsFact() const {
    return var_values_.size() == 1;
  }

  inline size_t GetSize() const {
    return var_values_.size();
  }

  inline void Clear() {
    var_values_.clear();
  }

  inline VarValue GetVarValue() const {
    return var_values_.at(0);
  }

  inline void AddVarValue(VarValue v) {
    auto result = std::find(var_values_.begin(), var_values_.end(), v);
    if (result != var_values_.end()) return;
    var_values_.push_back(v);
    std::sort(var_values_.begin(), var_values_.end());
  }

  inline const std::vector<VarValue>& GetVarValues() const {
    return var_values_;
  }

  size_t Hash() const;

  bool IsImplicated(const State &state) const;

  bool IsImplicated(const std::vector<VarValue> &assignment) const;

  bool IsImplicated(int var, int value) const;

  void Print() const;

 private:
  std::vector<VarValue> var_values_;
};

struct LandmarkHash {
  inline size_t operator()(const Landmark &landmark) const {
    return landmark.Hash();
  }
};

}

#endif // LANDMARK_H_
