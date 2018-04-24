#ifndef DOMAIN_H_
#define DOMAIN_H_

#include <string>
#include <unordered_set>
#include <vector>

#include "domain/state.h"
#include "domain/var_value.h"

namespace rwls {

class Preconditions {
 public
}

class Effects {
 public:
  explicit Effects(size_t size) : {
    begin_.reserve(size);
    end_.reserve(size);
    vars_.reserve(size);
    values_.reserve(size);
  }

  size_t size() const { return begin_.size(); }

  std::vector<int>::const_iterator vars_begin(int i) const {
    return vars_.begin() + begin_[i];
  }

  std::vector<int>::const_iterator vars_end(int i) const {
    return vars_.begin() + end_[i];
  }

  std::vector<int>::const_iterator values_begin(int i) const {
    return values_.begin() + begin_[i];
  }

  std::vector<int>::const_iterator values_end(int i) const {
    return values_.begin() + end_[i];
  }

  const int* offsets_data() const { return begin_.data(); }

  const int* sizes_data() const { return end_.data(); }

  const int* vars_data() const { return vars_.data(); }

  const int* values_data() const { return values_.data(); }

  void Apply() const;

  void AddEffect(const std::vector<int> &vars, const std::vector<int> &values);

 private:
  std::vector<int> begin_;
  std::vector<int> end_;
  std::vector<int> vars_;
  std::vector<int> values_;
};

class Problem {
 public:
 private:
  int metric_;
  size_t variables_size_;
  size_t action_size_;
  size_t fact_size_;
  State initial_;
  std::vector<int> dom_;
  std::vector<int> fact_offset_;
  std::vector<std::string> fact_to_predicate_;
  std::vector< std::unordered_set<VarValue> > mutex_groups_;
  std::vector<VarValue> goal_;
  std::vector<std::string> names_;
  std::vector<int> costs_;
  std::vector<int> preconditon_index_;
  std::vector<VarValue> preconditions_;
  std::vector<VarValue> effects_;
};

struct Domain {
  int metric;
  size_t variables_size;
  size_t action_size;
  size_t fact_size;
  State initial;
  std::vector<int> dom;
  std::vector<int> fact_offset;
  std::vector<std::string> fact_to_predicate;
  std::vector< std::unordered_set<VarValue> > mutex_groups;
  std::vector<VarValue> goal;
  std::vector<std::string> names;
  std::vector<int> costs;
  std::vector< std::vector<VarValue> > preconditions;
  std::vector< std::vector<VarValue> > effects;
};

inline int ToFact(const std::vector<int> &fact_offset, int var, int value) {
  return fact_offset[var] + value;
}

inline int ToFact(const std::vector<int> &fact_offset, VarValue var_value) {
  int var, value;
  DecodeVarValue(var_value, &var, &value);
  return ToFact(fact_offset, var, value);
}

std::vector<bool> ToFacts(const Domain &domain, const State &state);

void StateToFactSet(const State &state, const Domain &domain,
                    std::vector<int> &facts);

int HammingDistance(const std::vector<bool> &a, const std::vector<bool> &b);

} // namespace rwls

#endif // DOMAIN_H_
