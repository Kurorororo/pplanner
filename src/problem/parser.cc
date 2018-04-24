#include "domain/parser.h"

#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <unordered_set>

#include "domain/var_value.h"

using std::queue;
using std::string;
using std::unordered_set;
using std::vector;

namespace rwls {

const string kFileError = "invalid SAS file";
const string kAxiomError = "axiom is not supported";
const char kDelimiter = ' ';
const char kBra = '(';

int ParseVersion(queue<std::string> &lines) {
  if (lines.front() != "begin_version") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  int version = std::stoi(lines.front());
  lines.pop();
  if (lines.front() != "end_version") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  return version;
}

int ParseMetric(queue<string> &lines) {
  if (lines.front() != "begin_metric") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  int metric = std::stoi(lines.front());
  lines.pop();
  if (lines.front() != "end_metric") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  return metric;
}

std::string ParsePredicate(const std::string &line) {
  if (line == "<none of those>") return line;
  std::string buffer;
  std::istringstream line_separater(line);
  std::getline(line_separater, buffer, kDelimiter);
  std::getline(line_separater, buffer, kDelimiter);
  std::istringstream predicate_separater(buffer);
  std::getline(predicate_separater, buffer, kBra);
  return buffer;
}

int ParseVariable(queue<string> &lines, vector<string> &fact_to_predicate) {
  if (lines.front() != "begin_variable") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  if (lines.front().substr(0, 3) != "var") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  if (std::stoi(lines.front()) != -1) {
    std::cerr << kAxiomError << std::endl;
    exit(1);
  };
  lines.pop();
  int n = std::stoi(lines.front());
  int sup = n;
  lines.pop();
  for (int i=0; i<n; ++i) {
    fact_to_predicate.push_back(ParsePredicate(lines.front()));
    lines.pop();
  };
  if (lines.front() != "end_variable") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  return sup;
}

void ParseVariables(queue<string> &lines, Domain *domain) {
  int n = std::stoi(lines.front());
  domain->variables_size = static_cast<size_t>(n);
  lines.pop();
  domain->initial.resize(n);
  domain->dom.resize(n);
  domain->fact_offset.resize(n);
  int sum = 0;
  for (int i=0; i<n; ++i) {
    domain->fact_offset[i] = sum;
    int range = ParseVariable(lines, domain->fact_to_predicate);
    domain->dom[i] = range;
    sum += range;
  }
  domain->fact_size = static_cast<size_t>(sum);
}

void ParseVarValue(const string &line, VarValue *var_value) {
  std::string buffer;
  std::istringstream line_separater(line);
  std::getline(line_separater, buffer, kDelimiter);
  int var = std::stoi(buffer);
  std::getline(line_separater, buffer, kDelimiter);
  int value = std::stoi(buffer);
  EncodeVarValue(var, value, var_value);
}

void ParseMutexGroup(queue<std::string> &lines,
                     unordered_set<VarValue> &mutex_group) {
  if (lines.front() != "begin_mutex_group") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  int n = std::stoi(lines.front());
  lines.pop();
  for (int i=0; i<n; ++i) {
    VarValue v;
    ParseVarValue(lines.front(), &v);
    mutex_group.insert(v);
    lines.pop();
  }
  if (lines.front() != "end_mutex_group") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
}

void ParseMutexGroups(queue<string> &lines,
                      vector< unordered_set<VarValue> > &mutex_groups) {
  int n = std::stoi(lines.front());
  mutex_groups.resize(n);
  lines.pop();
  for (int i=0; i<n; ++i)
    ParseMutexGroup(lines, mutex_groups[i]);
}

void ParseState(queue<string> &lines, vector<int> &initial) {
  if (lines.front() != "begin_state") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  for (int i=0, n=initial.size(); i<n; ++i) {
    initial[i] = std::stoi(lines.front());
    lines.pop();
  }
  if (lines.front() != "end_state") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
}

void ParseGoal(queue<string> &lines, vector<VarValue> &goal) {
  if (lines.front() != "begin_goal") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  int n = std::stoi(lines.front());
  goal.resize(n);
  lines.pop();
  for (int i=0; i<n; ++i) {
    ParseVarValue(lines.front(), &goal[i]);
    lines.pop();
  }
  if (lines.front() != "end_goal") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
}

int ParsePrecondition(queue<string> &lines,
                      vector<VarValue> &precondition) {
  std::string buffer;
  int n = std::stoi(lines.front());
  precondition.resize(n);
  lines.pop();
  for (int i=0; i<n; ++i) {
   ParseVarValue(lines.front(), &precondition[i]);
   lines.pop();
  }
  return n;
}

void ParseEffect(queue<string> &lines, vector<VarValue> &precondition,
                 vector<VarValue> &effect) {
  int n = std::stoi(lines.front());
  effect.resize(n);
  lines.pop();
  string buffer;
  for (int i=0; i<n; ++i) {
    std::istringstream line_separater(lines.front());
    std::getline(line_separater, buffer, kDelimiter);
    if (std::stoi(buffer) != 0) {
      std::cerr << "conditional effect is not supported" << std::endl;
      exit(1);
    }
    std::getline(line_separater, buffer, kDelimiter);
    int var = std::stoi(buffer);
    std::getline(line_separater, buffer, kDelimiter);
    int value = std::stoi(buffer);
    if (value != -1) {
      int n = precondition.size();
      precondition.resize(n+1);
      EncodeVarValue(var, value, &precondition[n]);
    }
    std::getline(line_separater, buffer, kDelimiter);
    value = std::stoi(buffer);
    EncodeVarValue(var, value, &effect[i]);
    lines.pop();
  }
}

void ParseOperator(queue<string> &lines, int metric, string *name, int *cost,
                   vector<VarValue> &precondition,
                   vector<VarValue> &effect) {
  if (lines.front() != "begin_operator") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
  *name = lines.front();
  lines.pop();
  ParsePrecondition(lines, precondition);
  ParseEffect(lines, precondition, effect);
  if (metric == 0)
    *cost = 1;
  else
    *cost = std::stoi(lines.front());
  lines.pop();
  if (lines.front() != "end_operator") {
    std::cerr << kFileError << std::endl;
    exit(1);
  }
  lines.pop();
}

void ParseOperators(queue<string> &lines, Domain *domain) {
  int n = std::stoi(lines.front());
  domain->action_size = static_cast<size_t>(n);
  domain->names.resize(n);
  domain->costs.resize(n);
  domain->preconditions.resize(n);
  domain->effects.resize(n);
  lines.pop();
  for (int i=0; i<n; ++i)
    ParseOperator(lines, domain->metric, &domain->names[i], &domain->costs[i],
                  domain->preconditions[i], domain->effects[i]);
}

void Parse(const string &filename, Domain *domain) {
  std::ifstream input;
  input.open(filename, std::ios::in);
  std::string buffer;
  std::queue<std::string> lines;
  while (std::getline(input, buffer)) {
    lines.push(buffer);
  }
  input.close();
  int version = ParseVersion(lines);
  if (version != 3) {
    std::cerr << "SAS version must be 3" << std::endl;
    exit(1);
  }
  std::cout << "Parsing metric" << std::endl;
  domain->metric = ParseMetric(lines);
  std::cout << "Parsing variables" << std::endl;
  ParseVariables(lines, domain);
  std::cout << "Parsing mutex groups" << std::endl;
  ParseMutexGroups(lines, domain->mutex_groups);
  std::cout << "Parsing initial state" << std::endl;
  ParseState(lines, domain->initial);
  std::cout << "Parsing goal" << std::endl;
  ParseGoal(lines, domain->goal);
  std::cout << "Parsing operators" << std::endl;
  ParseOperators(lines, domain);
  if (std::stoi(lines.front()) != 0) {
    std::cerr << kAxiomError << std::endl;
    exit(1);
  }
  std::cout << "Parse succeed!" << std::endl;
}

} // namespace rwls
