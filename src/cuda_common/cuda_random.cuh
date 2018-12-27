#ifndef CUDA_RNDOM_H_
#define CUDA_RNDOM_H_

#include <curand_kernel.h>

namespace pplanner {

__global__
void SetupStates(unsigned long long seed, curandState_t *d_states);

} // namespace pplanner

#endif // CUDA_RNDOM_H_
