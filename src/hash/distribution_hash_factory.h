#ifndef DISTRIBUTION_HASH_FACTORY_H_
#define DISTRIBUTION_HASH_FACTORY_H_

#include <cstdint>

#include <memory>
#include <string>

#include "sas_plus.h"
#include "hash/distribution_hash.h"

namespace pplanner {

std::shared_ptr<DistributionHash> DistributionHashFactory(
    std::shared_ptr<const SASPlus> problem,
    uint32_t seed,
    const std::string &abstraction);

} // namespace pplanner

#endif // DISTRIBUTION_HASH_FACTORY_H_
