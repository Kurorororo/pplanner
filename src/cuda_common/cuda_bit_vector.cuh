#ifndef CUDA_BIT_VECTOR_H_
#define CUDA_BIT_VECTOR_H_

namespace pplanner {

__device__
inline bool CudaGet(const uint8_t *bits, int i) {
  return (bits[i / 8] & (1 << (i % 8))) != 0;
}

__device__
inline void CudaUp(uint8_t *bits, int i) { bits[i / 8] |= 1 << (i % 8); }

__device__
inline void CudaDown(uint8_t *bits, int i) { bits[i / 8] &= ~(1 << (i % 8)); }

} // namespace pplanner

#endif // CUDA_BIT_VECTOR_H_

