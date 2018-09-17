#include "sas_plus/parse_utils.h"

#include "sas_plus/facts.h"

#include <sstream>

using std::pair;
using std::queue;
using std::string;
using std::vector;

namespace pplanner {

const string kFormatError = "Exception: invalid SAS file";
const string kAxiomNotSupported = "Exceptin: axiom is not supported";
const string kConditionalEffectNotSupported = "Exceptin: conditional effect is not supported";

inline void CheckString(const string &expected, queue<string> &lines) {
  auto line = lines.front();
  lines.pop();

  if (expected != line)
    throw std::runtime_error(kFormatError);
}

int ParseN(queue<string> &lines) {
  int n = std::stoi(lines.front());
  lines.pop();

  return n;
}

int ParseVersion(queue<string> &lines) {
  CheckString("begin_version", lines);
  int version = ParseN(lines);
  CheckString("end_version", lines);

  return version;
}

int ParseMetric(queue<string> &lines) {
  CheckString("begin_metric", lines);
  int metric = ParseN(lines);
  CheckString("end_metric", lines);

  return metric;
}

std::string ParsePredicate(queue<string> &lines) {
  string line = lines.front();
  lines.pop();

  if (line == "<none of those>") return line;

  std::string buffer;
  std::istringstream line_separater(line);
  std::getline(line_separater, buffer, ' ');
  std::getline(line_separater, buffer, ' ');
  std::istringstream predicate_separater(buffer);
  std::getline(predicate_separater, buffer, '(');

  return buffer;
}

vector<string> ParseVariable(queue<string> &lines) {
  CheckString("begin_variable", lines);

  auto line = lines.front().substr(0, 3);
  lines.pop();
  if (line != "var") throw kFormatError;

  int n_axiom = ParseN(lines);
  if (n_axiom != -1) throw kAxiomNotSupported;

  int n = ParseN(lines);
  vector<string> predicates;
  predicates.reserve(n);

  for (int i=0; i<n; ++i)
    predicates.push_back(ParsePredicate(lines));

  CheckString("end_variable", lines);

  return predicates;
}

pair<int, int> ParseVarValue(queue<string> &lines) {
  string line = lines.front();
  lines.pop();

  std::string buffer;
  std::istringstream line_separater(line);
  std::getline(line_separater, buffer, ' ');
  int var = std::stoi(buffer);
  std::getline(line_separater, buffer, ' ');
  int value = std::stoi(buffer);

  return std::make_pair(var, value);
}

vector<int> ParseMutexGroup(const Facts &facts, queue<std::string> &lines) {
  CheckString("begin_mutex_group", lines);

  int n = ParseN(lines);
  std::vector<int> group;
  group.reserve(n);

  for (int i=0; i<n; ++i) { auto var_value = ParseVarValue(lines);
    int f = facts.Fact(var_value.first, var_value.second);
    group.push_back(f);
  }

  CheckString("end_mutex_group", lines);

  return group;
}

vector<int> ParseInitial(const Facts &facts, queue<string> &lines) {
  CheckString("begin_state", lines);

  size_t n = facts.n_variables();
  vector<int> initial(n);

  for (size_t i=0; i<n; ++i)
    initial[i] = ParseN(lines);

  CheckString("end_state", lines);

  return initial;
}

vector<pair<int, int> > ParseGoal(queue<string> &lines) {
  CheckString("begin_goal", lines);

  int n = ParseN(lines);
  vector<pair<int, int> > goal;
  goal.reserve(n);

  for (int i=0; i<n; ++i)
    goal.push_back(ParseVarValue(lines));

  CheckString("end_goal", lines);

  return goal;
}

void ParsePrecondition(queue<string> &lines,
                       vector<pair<int, int> > &precondition) {
  precondition.clear();
  int n = ParseN(lines);
  precondition.reserve(n);

  for (int i=0; i<n; ++i)
    precondition.push_back(ParseVarValue(lines));
}

void ParseEffect(queue<string> &lines, vector<pair<int, int> > &precondition,
                 vector<pair<int, int> > &effect,
                 vector<vector<pair<int, int> > > &effect_conditions,
                 vector<pair<int, int> > &conditional_effects) {
  effect.clear();
  effect_conditions.clear();
  conditional_effects.clear();
  int n = ParseN(lines);
  effect.reserve(n);
  string buffer;

  for (int i=0; i<n; ++i) {
    std::istringstream line_separater(lines.front());
    lines.pop();
    std::getline(line_separater, buffer, ' ');
    int n_ce = std::stoi(buffer);

    if (n_ce > 0) effect_conditions.push_back(vector<pair<int, int> >());

    for (int j=0; j<n_ce; ++j) {
      std::getline(line_separater, buffer, ' ');
      int var = std::stoi(buffer);
      std::getline(line_separater, buffer, ' ');
      int value = std::stoi(buffer);
      effect_conditions.back().push_back(std::make_pair(var, value));
    }

    std::getline(line_separater, buffer, ' ');
    int var = std::stoi(buffer);
    std::getline(line_separater, buffer, ' ');
    int value = std::stoi(buffer);

    if (value != -1) precondition.push_back(std::make_pair(var, value));

    std::getline(line_separater, buffer, ' ');
    value = std::stoi(buffer);

    if (n_ce == 0)
      effect.push_back(std::make_pair(var, value));
    else
      conditional_effects.push_back(std::make_pair(var, value));
  }
}

int ParseOperator(queue<string> &lines, int metric, string &name,
                  vector<pair<int, int> > &precondition,
                  vector<pair<int, int> > &effect,
                  vector<vector<pair<int, int> > > &effect_conditions,
                  vector<pair<int, int> > &conditional_effects) {
  CheckString("begin_operator", lines);

  name = lines.front();
  lines.pop();
  ParsePrecondition(lines, precondition);
  ParseEffect(lines, precondition, effect, effect_conditions,
              conditional_effects);
  int cost = ParseN(lines);
  if (metric == 0) cost = 1;

  CheckString("end_operator", lines);

  return cost;
}

void ParseAxiom(queue<string> &lines) {
  throw std::runtime_error(kAxiomNotSupported);
}

} // namespace pplanner
