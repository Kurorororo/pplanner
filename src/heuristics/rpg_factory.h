#ifndef RPG_FACTORY_H_
#define RPG_FACTORY_H_

#include <memory>

#include "heuristics/hn_rpg.h"
#include "heuristics/relaxed_sas_plus.h"
#include "heuristics/rpg.h"
#include "heuristics/rpg_table.h"
#include "sas_plus.h"

namespace pplanner {

inline std::shared_ptr<RPG> RPGFactory(
    std::shared_ptr<const SASPlus> problem,
    std::shared_ptr<const RelaxedSASPlus> r_problem, bool use_rpg_table,
    std::string tie_break = "cpp", bool more_helpful = false) {
  if (use_rpg_table)
    return std::make_shared<RPGTable>(problem, r_problem, tie_break,
                                      more_helpful);

  return std::make_shared<HNRPG>(r_problem);
}

};  // namespace pplanner

#endif  // RPG_FACTORY_H_"
