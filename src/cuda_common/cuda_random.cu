#include "cuda_common/cuda_random.cuh"

namespace pplanner {

__global__
void SetupStates(unsigned long long seed, curandState_t *d_states) {
  int id = threadIdx.x + blockIdx.x * blockDim.x;
  curand_init(seed, id, 0, &d_states[id]);
}

} // namespace pplanner
