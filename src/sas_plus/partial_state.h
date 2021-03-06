#ifndef PARTIAL_STATE_H_
#define PARTIAL_STATE_H_

#include <cstddef>

#include <utility>
#include <vector>

namespace pplanner {

class PartialState {
 public:
  PartialState() {}

  explicit PartialState(const std::vector<std::pair<int, int> > &v);

  int size() const { return vars_.size(); }

  void Add(int var, int value) {
    vars_.push_back(var);
    values_.push_back(value);
  }

  bool IsSubset(const std::vector<int> &state) const;

  void Copy(std::vector<std::pair<int, int> > &v) const;

  int Var(int i) const { return vars_[i]; }

  int Value(int i) const { return values_[i]; }

  void Dump() const;

  const int* vars_data() const { return vars_.data(); }

  const int* values_data() const { return values_.data(); }

 private:
  std::vector<int> vars_;
  std::vector<int> values_;
};

} // namespace pplanner

#endif // PARTIAL_STATE_H_
