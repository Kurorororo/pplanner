#ifndef HEURISTIC_H_
#define HEURISTIC_H_

#include <type_traits>

#include "domain/domain.h"
#include "domain/state.h"

namespace rwls {

template<class T>
class HeuristicInterface {
 public:
  void Initialize(const Domain &domain) {
    static_cast<T &>(this)->Initialize(domain);
  }

  int operator()(const State &state, const Domain &domain) {
    return static_cast<T &>(this)->operator()(state, domain);
  }

};

template<class T>
class Heuristic {
  static_assert(std::is_base_of<HeuristicInterface<T>, T>::value,
                "T is not extended interface class");
 public:
  void Initialize(const Domain &domain) {
    object_.Initialize(domain);
  }

  int operator()(const State &state, const Domain &domain) {
    return object_(state, domain);
  }

  T object_;
};

} // rwls

#endif // HEURISTIC_H_
