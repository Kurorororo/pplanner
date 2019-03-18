#ifndef RPG_H_
#define RPG_H_

#include <unordered_set>
#include <vector>

namespace pplanner {

class RPG {
 public:
  virtual ~RPG() = 0;

  virtual int PlanCost(const std::vector<int> &state) = 0;

  virtual int PlanCost(const std::vector<int> &state,
                       std::unordered_set<int> &helpful) = 0;

  virtual void ConstructRRPG(const std::vector<int> &state,
                             const std::vector<bool> &black_list) = 0;

  virtual void DisjunctiveHelpful(const std::vector<int> &state,
                                  const std::vector<int> &disjunctive_goal,
                                  std::unordered_set<int> &helpful) = 0;

  virtual bool IsIn(int fact) const = 0;

  virtual bool IsApplicable(int action) const = 0;
};

}  // namespace pplanner

#endif  // RPG_H_
