#ifndef PARSE_UTILS_H_
#define PARSE_UTILS_H_

#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "sas_plus/facts.h"

namespace pplanner {

int ParseN(std::queue<std::string> &lines);

int ParseVersion(std::queue<std::string> &lines);

int ParseMetric(std::queue<std::string> &lines);

std::vector<std::string> ParseVariable(std::queue<std::string> &lines);

std::vector<int> ParseMutexGroup(const Facts &facts,
                                 std::queue<std::string> &lines);

std::vector<int> ParseInitial(const Facts &facts,
                              std::queue<std::string> &lines);

std::vector<std::pair<int, int> > ParseGoal(std::queue<std::string> &lines);

int ParseOperator(
    std::queue<std::string> &lines,
    int metric,
    std::string &name,
    std::vector<std::pair<int, int> > &precondition,
    std::vector<std::pair<int, int> > &effect,
    std::vector<std::vector<std::pair<int, int> > > &effect_conditions,
    std::vector<std::pair<int, int> > &conditional_effects);

void ParseAxiom(std::queue<std::string> &lines);

} // namespace pplaner

#endif // PARSE_UTILS_H_
