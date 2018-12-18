#ifndef ARRAY_HASH_H_
#define ARRAY_HASH_H_

#include <cstdint>

#include <vector>

namespace pplanner {

/**
 * See also "Open Data Structures", 5.3.2
 */
template<typename T>
class ArrayHash {
 public:
  ArrayHash() {}

  ArrayHash(std::size_t size);

  uint32_t operator()(const T *array) const;

  uint32_t operator()(const std::vector<T> &array) const;

 private:
  uint64_t zz_;
  std::vector<uint64_t> z_;
  std::hash<T> hash_;
};

} // namespace pplanner

#include "./details/array_hash.h"

#endif // ARRAY_HASH_H_
