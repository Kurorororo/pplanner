#ifndef BIT_VECTOR_H_
#define BIT_VECTOR_H_

#include <vector>

namespace pplanner {

class BitVector {
 public:
  BitVector(size_t size) : bits_((size + 7) / 8, 0) {}

  bool Get(int i) const { return (bits_[i / 8] & (1 << (i % 8))) != 0; }

  void Up(int i) { bits_[i / 8] |= 1 << (i % 8); }

  void Down(int i) { bits_[i / 8] &= ~(1 << (i % 8)); }

 private:
  std::vector<uint8_t> bits_;
};

inline bool Get(const uint8_t *bits, int i) {
  return (bits[i / 8] & (1 << (i % 8))) != 0;
}

inline void Up(uint8_t *bits, int i) { bits[i / 8] |= 1 << (i % 8); }

inline void Down(uint8_t *bits, int i) { bits[i / 8] &= ~(1 << (i % 8)); }

}

#endif // BIT_VECTOR_H_
