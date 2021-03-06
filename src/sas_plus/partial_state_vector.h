#ifndef PARTIAL_STATE_VECTORS_H_
#define PARTIAL_STATE_VECTORS_H_

#include <cassert>
#include <cstddef>

#include <vector>

namespace pplanner {

class PartialStateVector {
 public:
  PartialStateVector() { offsets_.push_back(0); }

  virtual ~PartialStateVector() {}

  virtual void Dump(int i) const;

  void Reserve(size_t size) {
    offsets_.reserve(size);
    vars_.reserve(size);
    values_.reserve(size);
  }

  int size() const { return offsets_.size() - 1; }

  std::size_t n_units() const { return vars_.size(); }

  int SizeOfPartialState(int i) const {
    assert(size() > 0);

    return offsets_[i + 1] - offsets_[i];
  }

  std::vector<int>::const_iterator VarsBegin(int i) const {
    return vars_.begin() + offsets_[i];
  }

  std::vector<int>::const_iterator VarsEnd(int i) const {
    assert(size() > 0);

    return vars_.begin() + offsets_[i + 1];
  }

  std::vector<int>::const_iterator ValuesBegin(int i) const {
    return values_.begin() + offsets_[i];
  }

  std::vector<int>::const_iterator ValuesEnd(int i) const {
    assert(size() > 0);

    return values_.begin() + offsets_[i + 1];
  }

  void Copy(int i, std::vector<std::pair<int, int> > &v) const;

  void Add(const std::vector<std::pair<int, int> > &v);

  const int* offsets_data() const { return offsets_.data(); }

  const int* vars_data() const { return vars_.data(); }

  const int* values_data() const { return values_.data(); }

 private:
  std::vector<int> offsets_;
  std::vector<int> vars_;
  std::vector<int> values_;
};

} // namespace pplanner

#endif // PARTIAL_STATE_VECTORS_H_
