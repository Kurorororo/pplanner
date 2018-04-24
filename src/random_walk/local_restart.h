#ifndef LOCAL_RESTART_H_
#define LOCAL_RESTART_H_

#include <random>
#include <vector>

namespace rwls {

void Feedback(int arg_rl, int value, int cost, std::vector<int> &value_rls,
              std::vector<int> &cost_rls);

int ChoiceRl(double eps, const std::vector<int> &value_rls,
             const std::vector<int> &cost_rls,
             std::default_random_engine &engine);

template<typename T>
T GetRl(int arg_rl, const std::vector<T> &rls) {
  return rls[arg_rl];
}

} // namespace rwls

#endif // LOCAL_RESTART_H_
