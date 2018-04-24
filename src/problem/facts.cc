#include "private/facts.h"

using std::vector;

namespace pplanner {

void StateToFactVector(const FactTranslator &translater,
                       const vector<int> &state, vector<int> &facts) {
  facts.clear();

  int i = 0;

  for (auto value : state) {
    int f = translater.ToFact(i, value);
    facts.push_back(f);
    ++i;
  }
}

void StateToFactSet(const FactTranslator &translater, const vector<int> &state,
                    vector<bool> &facts) {
  facts.resize(translater.n_facts(), false);

  int i = 0;

  for (auto value : state) {
    int f = translater.ToFact(i, value);
    facts[f] = true;
    ++i;
  }

} // namespace pplanner
