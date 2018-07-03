#include <functional>
#include <random>

namespace pplanner {

template<typename T>
ArrayHash<T>::ArrayHash(size_t size) : z_(size) {
  // these can be randomized by using std::random_device.
  std::mt19937 engine(1395437481);
  std::mt19937_64 engine64(866667759);

  zz_ = engine64();

  for (auto &v : z_)
    v = static_cast<uint64_t>(engine());
}

template<typename T>
uint32_t ArrayHash<T>::operator()(const T *array) const {
  uint64_t hash = 0;

  for (size_t i=0, n=z_.size(); i<n; ++i)
    hash += z_[i] * static_cast<uint64_t>(hash_(array[i]));

  return static_cast<uint32_t>((zz_ * hash) >> 32);
}

template<typename T>
uint32_t ArrayHash<T>::operator()(const std::vector<T> &array) const {
  uint64_t hash = 0;

  for (size_t i=0, n=z_.size(); i<n; ++i)
    hash += z_[i] * static_cast<uint64_t>(hash_(array[i]));

  return static_cast<uint32_t>((zz_ * hash) >> 32);
}

} // namespace pplanner
