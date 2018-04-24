#include "domain/domain.h"

namespace rwls {

std::vector<bool> ToFacts(const Domain &domain, const State &state) {
  std::vector<bool> facts(domain.fact_size, false);

  for (size_t i=0, n=domain.variables_size; i<n; ++i)
    facts[ToFact(domain.fact_offset, i, state[i])] = true;

  return facts;
}

void StateToFactSet(const State &state, const Domain &domain,
                    std::vector<int> &facts) {
  facts.clear();

  for (int i=0, n=state.size(); i<n; ++i)
    facts.push_back(ToFact(domain.fact_offset, i, state[i]));
}

int HammingDistance(const std::vector<bool> &a, const std::vector<bool> &b) {
  int d = 0;

  for (size_t i=0, n=a.size(); i<n; ++i)
    if (a[i] != b[i]) ++d;

  return d;
}


} // namespace rwls
