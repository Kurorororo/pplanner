#include "sas_plus/facts.h"

using std::vector;

namespace pplanner {

int Facts::AddVariable(const vector<std::string> &predicates) {
  predicates_.insert(predicates_.end(), predicates.begin(), predicates.end());
  int range = static_cast<int>(predicates.size());
  size_ += range;
  ranges_.push_back(range);
  offsets_.push_back(offsets_.back() + range);

  return static_cast<int>(n_variables_++);
}

void StateToFactVector(const Facts &facts, const vector<int> &state,
                       vector<int> &v) {
  v.clear();

  int i = 0;

  for (auto value : state) {
    int f = facts.Fact(i, value);
    v.push_back(f);
    ++i;
  }
}

void StateToFactSet(const Facts &facts, const vector<int> &state,
                    vector<bool> &s) {
  s.resize(facts.size(), false);

  int i = 0;

  for (auto value : state) {
    int f = facts.Fact(i, value);
    s[f] = true;
    ++i;
  }
}

} // namespace pplanner
