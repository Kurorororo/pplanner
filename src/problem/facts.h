#ifndef FACTS_H_
#define FACTS_H_

#include <vector>

namespace pplanner {

class FactTranslator {
 public:
  FactTranslator() : n_facts_(0) { offsets_.push_back(0); }

  explicit FactTranslator(size_t size) : n_facts_(0) {
    offsets_.reserve(size);
    ranges_.reserve(size);
    offsets_.push_back(0);
  }

  size_t n_facts() const { return n_facts; }

  void Add(int range) {
    n_facts += range;
    ranges_.push_back(range);
    offsets_.push_back(offsets_.back() + ranges_.back());
  }

  int ToFact(int var, int value) { return offsets_[var] + value; }

  const int* offsets_data() const { return offsets_.data(); }

  const int* ranges_data() const { return ranges_.data(); }

 private:
  size_t n_facts_;
  std::vector<int> offsets_;
  std::vector<int> ranges_;
};

void StateToFactVector(const FactTranslator &translater,
                       const std::vector<int> &state, std::vector<int> &facts);

void StateToFactSet(const FactTranslator &translater,
                    const std::vector<int> &state, std::vector<bool> &facts);

inline std::vector<int> StateToFactVector(const FactTranslator &translater,
                                          const std::vector<int> &state) {
  std::vector<int> facts(state);
  StateToFactVector(translater, state, facts);

  return facts;
}

inline std::vector<bool> StateToFactSet(const FactTranslator &translater,
                                        const std::vector<int> &state) {
  std::vector<bool> facts(translater.n_facts(), false);
  StateToFactSet(translater, state, facts);

  return facts;
}

}

#endif FACTS_H_
