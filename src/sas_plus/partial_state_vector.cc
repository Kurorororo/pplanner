#include "sas_plus/partial_state_vector.h"

using std::pair;
using std::vector;

namespace pplanner {

void PartialStateVector::Add(const vector<pair<int, int> > &v) {
  size_t size = v.size();

  for (auto f : v) {
    vars_.push_back(f.first);
    values_.push_back(f.second);
  }

  offsets_.push_back(offsets_.back() + static_cast<int>(size));
}

} // namespace pplanner
