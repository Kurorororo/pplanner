#include "random_walk/local_restart.h"

using std::vector;

namespace rwls {

void Feedback(int arg_rl, int value, int cost, vector<int> &value_rls,
              vector<int> &cost_rls) {
  value_rls[arg_rl] += value;
  cost_rls[arg_rl] += cost;
}

int RandomRl(size_t size, std::default_random_engine &engine) {
  return engine() % size;
}

int ChoiceRl(double eps, const vector<int> &value_rls,
             const vector<int> &cost_rls, std::default_random_engine &engine) {
  std::uniform_real_distribution<> dist(0.0, 1.0);
  if (dist(engine) < eps) return RandomRl(value_rls.size(), engine);

  double q_max = 0.0;
  int arg_best = -1;

  for (int i=0, n=cost_rls.size(); i<n; ++i) {
    if (cost_rls[i] == 0) continue;

    double q = static_cast<double>(value_rls[i]);
    q /= static_cast<double>(cost_rls[i]);

    if (q > q_max) {
      q_max = q;
      arg_best = i;
    }
  }

  if (arg_best == -1) return RandomRl(value_rls.size(), engine);

  return arg_best;
}

} // namespace rwls
