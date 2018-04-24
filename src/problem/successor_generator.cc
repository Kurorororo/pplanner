#include "problem/successor_generator.h"

#include <algorithm>
#include <random>

using std::vector;

namespace pplanner {

void InitializeTable(const Domain &domain, TrieTable *table) {
  size_t max_children = domain.fact_size;
  table->size = max_children;
  table->data_size = 0;
  table->data_size_array = NULL;
  table->data = NULL;
  table->to_child = (int*)malloc(max_children*sizeof(int));
  ALLOC_CHECK(table->to_child);
  table->to_data = (int*)malloc(max_children*sizeof(int));
  ALLOC_CHECK(table->to_data);
  memset(table->to_child, -1, max_children*sizeof(int));
  memset(table->to_data, -1, max_children*sizeof(int));
}

void FinalizeTable(TrieTable *table) {
  free(table->to_child);
  free(table->to_data);
  if (table->data_size_array != NULL) free(table->data_size_array);
  if (table->data != NULL) free(table->data);
}

void AddQuery(int index, int query, TrieTable *table) {
  if (table->to_data[index] == -1) {
    table->to_data[index] = table->data_size++;
    if (table->data == NULL) {
      table->data_size_array = (size_t*)malloc(sizeof(size_t));
      table->data = (int**)malloc(sizeof(int*));
    } else {
      table->data_size_array = (size_t*)realloc(
                                   table->data_size_array,
                                   (table->data_size+1)*sizeof(size_t));
      table->data = (int**)realloc(table->data,
                                   (table->data_size+1)*sizeof(int*));
    }
    ALLOC_CHECK(table->data_size_array);
    ALLOC_CHECK(table->data);
    table->data[table->to_data[index]] = NULL;
    table->data_size_array[table->to_data[index]] = 0;
  }
  int data_index = table->to_data[index];
  int tail = table->data_size_array[data_index]++;
  if (table->data[data_index] == NULL) {
    table->data[data_index] = (int*)malloc(sizeof(int));
  } else {
    size_t new_size = table->data_size_array[data_index] * sizeof(int);
    table->data[data_index] = (int*)realloc(table->data[data_index], new_size);
  }
  ALLOC_CHECK(table->data[data_index]);
  table->data[data_index][tail] = query;
}

void AddQuery(int index, int query) {
  if (to_data_[index] == -1) {
    to_data_[index] = data_size_++;
    data_.push_back();


  }

  int data_index = to_data_[index];
}

void InsertToTable(const Domain &domain, int query,
                   vector<VarValue> precondition, TrieTable *table) {
  int i = 0;
  int j = 0;
  int max = precondition.size() - 1;
  int parent_prefix = 0;
  size_t max_children = domain.fact_size;
  std::sort(precondition.begin(), precondition.end());
  for (auto v : precondition) {
    int var, value;
    DecodeVarValue(v, &var, &value);
    int index = j + domain.fact_offset[var] - parent_prefix + value;
    if (i == max) {
      AddQuery(index, query, table);
      return;
    }
    parent_prefix = domain.fact_offset[var+1];
    if (table->to_child[index] == -1) {
      table->to_child[index] = table->size;
      int old_size = table->to_child[index];
      int new_size = old_size + max_children - parent_prefix;
      table->to_child = (int*)realloc(table->to_child, new_size*sizeof(int));
      ALLOC_CHECK(table->to_child);
      table->to_data = (int*)realloc(table->to_data, new_size*sizeof(int));
      ALLOC_CHECK(table->to_data);
      memset(&table->to_child[old_size], -1,
             (max_children-parent_prefix)*sizeof(int));
      memset(&table->to_data[old_size], -1,
             (max_children-parent_prefix)*sizeof(int));
      table->size = new_size;
    }
    j = table->to_child[index];
    ++i;
  }
}

TrieTable ConstructTable(const Domain &domain) {
  TrieTable table;
  InitializeTable(domain, &table);
  int n = static_cast<int>(domain.preconditions.size());
  for (int i=0; i<n; ++i)
    InsertToTable(domain, i, domain.preconditions[i], &table);
  return table;
}

void RecursiveFind(const TrieTable &table, const Domain &domain,
                   const State &state, int index, size_t current,
                   vector<int> &result) {
  int prefix = index - domain.fact_offset[current];
  for (size_t i=current, n=domain.variables_size; i<n; ++i) {
    int next = domain.fact_offset[i] + state[i] + prefix;
    int offset = table.to_data[next];
    if (offset != -1) {
      size_t size = result.size();
      size_t add_size = table.data_size_array[offset];
      result.resize(size + add_size);
      for (size_t j=0; j<add_size; ++j)
        result[size + j] = table.data[offset][j];
    }
    int child = table.to_child[next];
    if (child == -1) continue;
    RecursiveFind(table, domain, state, child, i+1, result);
  }
}

vector<int> FindFromTable(const TrieTable &table, const Domain &domain,
                          const State &state) {
  vector<int> result;
  RecursiveFind(table, domain, state, 0, 0, result);
  return result;
}

void FindFromTable(const TrieTable &table, const Domain &domain,
                   const State &state, vector<int> &result) {
  result.clear();
  RecursiveFind(table, domain, state, 0, 0, result);
}

void RecursiveSample(const TrieTable &table, const Domain &domain,
                     const State &state, int index, size_t current,
                     unsigned int *k, std::mt19937 &engine, int *result) {
  int prefix = index - domain.fact_offset[current];
  for (size_t i=current, n=state.size(); i<n; ++i) {
    int next = domain.fact_offset[i] + state[i] + prefix;
    int offset = table.to_data[next];
    if (offset != -1) {
      size_t size = table.data_size_array[offset];
      for (size_t j=0; j<size; ++j) {
        if (engine() % *k == 0)
          *result = table.data[offset][j];
        ++(*k);
      }
    }
    int child = table.to_child[next];
    if (child == -1) continue;
    RecursiveSample(table, domain, state, child, i+1, k, engine, result);
  }
}

int SampleFromTable(const TrieTable &table, const Domain &domain,
                    const State &state) {
  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());

  int result = -1;
  unsigned int k = 1;
  RecursiveSample(table, domain, state, 0, 0, &k, engine, &result);

  return result;
}

void PrintTable(const TrieTable &table) {
  for (size_t i=0; i<table.size; ++i)
    printf("%d ", table.to_child[i]);
  printf("\n");
  for (size_t i=0; i<table.size; ++i)
    printf("%d ", table.to_data[i]);
  printf("\n");
  for (size_t i=0; i<table.data_size; ++i) {
    for (size_t j=0; j<table.data_size_array[i]; ++j)
      printf("%d ", table.data[i][j]);
  }
  printf("\n");
}

} // namespace rwls
