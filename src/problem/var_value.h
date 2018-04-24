#ifndef VAR_VALUE_H_
#define VAR_VALUE_H_

#include <cstdint>

namespace rwls {

using VarValue = uint64_t;

inline void EncodeVarValue(int var, int value, VarValue *var_value) {
  *var_value = (static_cast<uint64_t>(var) << 32)
               ^ static_cast<uint64_t>(value);
}

inline int GetVar(VarValue var_value) {
  return static_cast<int>(var_value >> 32);
}

inline int GetValue(VarValue var_value) {
  return static_cast<int>(var_value & 0xFFFFFFFF);
}

inline void DecodeVarValue(VarValue var_value, int *var, int *value) {
  *var = static_cast<int>(var_value >> 32);
  *value = static_cast<int>(var_value & 0xFFFFFFFF);
}

} // namespace rwls

#endif // VAR_VALUE_H_
