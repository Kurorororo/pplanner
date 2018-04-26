#ifndef PARSE_UTILS_H_
#define PARSE_UTILS_H_

#include <iostream>
#include <queue>
#include <utility>
#include <vector>

#include "sas_plus/facts.h"

namespace pplaner {

int ParseN(queue<string> &lines);

int ParseVersion(queue<string> &lines);

int ParseMetric(queue<string> &lines);

std::string ParsePredicate(const std::string &line);

std::vector<std::string> ParseVariable(std::queue<std::string> &lines);

std::pair<int, int> ParseVarValue(const std::string &line);

std::vector<int> ParseMutexGroup(const Facts &facts,
                                 std::queue<std::string> &lines);

std::vector<int> ParseInitial(const Facts &fact, std::queue<std::string> &lines);

std::vector<std::pair<int, int> > ParseGoal(std::queue<std::string> &lines);

int ParseOperator(std::queue<std::string> &lines, std::string &name,
                  std::vector<std::pair<int, int> > &precondition,
                  std::vector<std::pair<int, int> > &effect);

void ParseAxiom(std::queue<std::string> &lines);

} // namespace pplaner

#endif // PARSE_UTILS_H_
