#ifndef CUDA_GBFS_KERNEL_H_
#define CUDA_GBFS_KERNEL_H_

#include "cuda_search_graph.cuh"
#include "sas_plus.h"
#include "search_graph/search_graph_with_landmarks.h"

namespace pplanner {

struct GBFSMessage {
  int n_successors_max;
  int n_nodes;
  // number of threads
  int *nodes;
  int *successor_counts;
  int *successor_offsets;
  int *received_counts;
  int *received_offsets;
  int *h_min;
  int *states;
  int *child_states;
  uint8_t *accepted;
  uint8_t *status;
  uint32_t *packed;
  // number of successors
  int *actions;
  int *successors;
  int *h;
  int *procs;
  int *received_indices;
  int *sorted;
  int *sorted_h;
};

struct CudaOpenList {
  std::size_t n_unit;
  int *next;
  int *prev;
};

struct CudaClosedList {
  std::size_t n_unit;
  int *closed;
};

__global__
void CudaInitialPush(CudaSearchGraph graph, GBFSMessage m, CudaOpenList open,
                     int n_thread);

__global__
void CudaInitialEvaluate(CudaSearchGraph graph, GBFSMessage m, CudaOpenList open,
                         int n_thread, int *h);

__global__
void CudaPop(CudaSearchGraph graph, GBFSMessage m, CudaOpenList open,
             CudaClosedList closed);

int PrepareExpansion(int threads, GBFSMessage *m, GBFSMessage *cuda_m);

__global__
void CudaExpand(CudaSearchGraph graph, GBFSMessage m, int n_threads, int *goal);

int PrepareSort(int threads, GBFSMessage *m, GBFSMessage *cuda_m);

__global__
void CudaSortChildren(const CudaSearchGraph graph, GBFSMessage m);

__global__
void CudaPush(CudaSearchGraph graph, GBFSMessage m, CudaClosedList closed,
              CudaOpenList open);

__global__
void CudaNPlanStep(const CudaSearchGraph graph, int *goals, int *steps);

__global__
void CudaExtractPlan(const CudaSearchGraph graph, int *offsets, int *goals,
                     int *plan);

void InitializeGBFSMessage(int threads, GBFSMessage *m);

void FreeGBFSMessage(GBFSMessage *m);

std::size_t CudaInitializeGBFSMessage(
    std::shared_ptr<const SASPlus> problem,
    std::shared_ptr<const SearchGraphWithLandmarks> graph,
    int threads,
    int n_successors_max,
    GBFSMessage *m);

void CudaFreeGBFSMessage(GBFSMessage *m);

void CudaReallocMessage(int size, GBFSMessage *message);

std::size_t CudaInitializeOpenList(int threads, std::size_t size,
                                   CudaOpenList *open);

void CudaClearOpenList(int threads, CudaOpenList *open);

void CudaFreeOpenList(CudaOpenList *open);

std::size_t CudaInitializeClosedList(int threads, std::size_t size,
                                     CudaClosedList *closed);

void CudaClearClosedList(int threads, CudaClosedList *closed);

void CudaFreeClosedList(CudaClosedList *closed);

} // namespace pplanner

#endif // CUDA_GBFS_KERNEL_H_
