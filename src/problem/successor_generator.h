#ifndef SUCCESSOR_GENERATOR_H_
#define SUCCESSOR_GENERATOR_H_

#include <vector>

namespace pplanner {

class SuccessorGenerator {
 public:
  explicit SuccessorGenerator(size_t fact_size) : size_(fact_size),
                                                  data_size_(0) {
    to_child_.resize(fact_size, -1);
    to_data_.resize(fact_size, -1);
  }

  void AddQuery(int index, int query);

  void Find(const std::vector<int> &state, std::vector<int> &result) const;

  int Sample(const std::vector<int> &state) const;

 private:
  size_t size_;
  size_t data_size_;
  std::vector<int> to_child_;
  std::vector<int> to_data_;
  std::vector<int> offsets_;
  std::vector<int> data_;
};

void InitializeTable(const Domain &domain, TrieTable *table);

void FinalizeTable(TrieTable *table);

void InsertToTable(const Domain &domain, int query,
                   std::vector<VarValue> precondition, TrieTable *table);

TrieTable ConstructTable(const Domain &domain);

std::vector<int> FindFromTable(const TrieTable &table, const Domain &domain,
                               const State &state);

void FindFromTable(const TrieTable &table, const Domain &domain,
                   const State &state, std::vector<int> &result);

int SampleFromTable(const TrieTable &table, const Domain &domain,
                    const State &state);

void PrintTable(const TrieTable &table);

} // namespace pplanner

#endif // SUCCESSOR_GENERATOR_H_
