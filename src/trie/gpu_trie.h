#ifndef GPU_TRIE_H_
#define GPU_TRIE_H_

#include <curand_kernel.h>

#include "common/gpu_vector.h"
#include "domain/device_domain.h"
#include "trie/trie.h"

namespace rwls {

// result can not be transported to host because FindFromTable calls malloc()
__device__
void FindFromTable(const TrieTable &table, const DeviceDomain &domain,
                   const int *variables, GVector<int> &result);

__device__
void FindFromTable(const TrieTable &table, const DeviceDomain &domain,
                   const int *variables, int *result);
__device__
int SampleFromTable(const TrieTable &table, const DeviceDomain &domain,
                    const int *variables, curandState_t *state);

__device__
int CountFromTable(const TrieTable &table, const DeviceDomain &domain,
                   const int *variables);

__device__
void DevicePrintTable(const TrieTable &table);

void CudaInitializeTable(const TrieTable &h_table, TrieTable *d_table);

void CudaFinalizeTable(TrieTable *d_table);

} // namespace rwls

#endif // GPU_TRIE_H_
