#ifndef FACTS_H_
#define FACTS_H_

#include <string>
#include <vector>

namespace pplanner {

class Facts {
 public:
  Facts() : size_(0), n_variables_(0) { offsets_.push_back(0); }

  size_t size() const { return size_; }

  size_t n_variables() const { return n_variables_; }

  void Reserve(size_t n_variables) {
    offsets_.reserve(n_variables);
    ranges_.reserve(n_variables);
    predicates_.reserve(n_variables);
  }

  int AddVariable(const std::vector<std::string> &predicates);

  int VarBegin(int var) const { return offsets_[var]; }

  int VarRange(int var) const { return ranges_[var]; }

  int Fact(int var, int value) const { return VarBegin(var) + value; }

  const std::string& Predicate(int var, int value) const {
    return predicates_[Fact(var, value)];
  }

  const int* offsets_data() const { return offsets_.data(); }

  const int* ranges_data() const { return ranges_.data(); }

 private:
  size_t size_;
  size_t n_variables_;
  std::vector<int> offsets_;
  std::vector<int> ranges_;
  std::vector<std::string> predicates_;
};

void StateToFactVector(const Facts &facts, const std::vector<int> &state,
                       std::vector<int> &v);

void StateToFactSet(const Facts &facts, const std::vector<int> &state,
                    std::vector<bool> &s);

} // namespace pplanner

#endif // FACTS_H_
