#include "partial_state_vector.h"

using std::vector;

namespace pplanner {

void PartialStateVector::Add(const vector<int> &vars,
                             const vector<int> &values) {
  size_t size = vars.size();

  assert(vars.size() == values.size());
  assert(size > 0);

  for (size_t i=0; i<size; ++i) {
    vars_.push_back(vars[i]);
    values_.push_back(values[i]);
  }

  offsets_.push_back(offsets.back() + static_cast<int>(size));
}

} // namespace pplanner
