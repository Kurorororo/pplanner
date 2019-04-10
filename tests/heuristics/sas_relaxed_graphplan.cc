#include "heuristics/hn_rpg.h"

#include <algorithm>
#include <iostream>
#include <unordered_set>
#include <vector>

#include <boost/program_options.hpp>

#include "heuristics/relaxed_sas_plus.h"
#include "sas_plus.h"
#include "utils/file_utils.h"

using namespace pplanner;

int main(int argc, char *argv[]) {
  namespace po = boost::program_options;
  po::options_description opt("Options");
  opt.add_options()("help,h", "help")("file,f", po::value<std::string>(),
                                      "input sas+ file name")(
      "simplify,s", "simplify unary operators");
  po::variables_map vm;

  try {
    po::store(po::parse_command_line(argc, argv, opt), vm);
  } catch (const po::error_with_option_name &e) {
    std::cout << e.what() << std::endl;
  }

  po::notify(vm);

  if (vm.count("help") || !vm.count("file")) {
    std::cout << opt << std::endl;
    exit(0);
  }

  auto filename = vm["file"].as<std::string>();
  bool simplify = vm.count("simplify");

  auto lines = FileToLines(filename);
  auto sas = std::make_shared<SASPlus>();
  sas->InitFromLines(lines);

  auto r_sas = std::make_shared<RelaxedSASPlus>(sas, simplify);
  HNRPG rpg(r_sas);
  std::unordered_set<int> preferred;

  auto initial = sas->initial();
  std::vector<int> initial_facts;
  StateToFactVector(sas, initial, initial_facts);

  auto result = rpg.Plan(initial_facts, preferred);
  int result_cost = rpg.PlanCost(initial_facts, preferred);
  std::cout << "plan" << std::endl;
  int cost = 0;

  for (auto o : result) {
    if (o == -1) {
      std::cerr << "faild to solve problem" << std::endl;
      exit(0);
    }

    cost += sas->ActionCost(o);
    std::cout << "number " << o << " " << sas->ActionName(o) << std::endl;
  }

  if (result_cost != cost) {
    std::cerr << "cost of plan and plan cost do not match" << std::endl;
    exit(1);
  }

  std::cout << std::endl;

  std::cout << "plan step: " << result.size() << std::endl;
  std::cout << "plan cost: " << cost << std::endl;
  std::cout << std::endl;

  std::cout << "preferred" << std::endl;

  for (auto o : preferred) std::cout << sas->ActionName(o) << std::endl;

  std::cout << std::endl;

  std::cout << "test" << std::endl;
  std::vector<int> facts(sas->n_facts(), 0);
  std::cout << "ini: ";

  for (int i = 0, n = initial.size(); i < n; ++i) {
    int f = sas->Fact(i, initial[i]);
    std::cout << "fact" << f << "|";
    std::cout << "var" << i << "=" << initial[i] << ", ";
    facts[f] = 1;
  }

  std::cout << std::endl << std::endl;

  for (auto o : result) {
    std::cout << sas->ActionName(o) << std::endl;
    std::cout << "number " << o << std::endl;
    std::cout << "pre: ";

    std::vector<std::pair<int, int> > precondition;
    sas->CopyPrecondition(o, precondition);

    for (auto p : precondition) {
      int f = sas->Fact(p.first, p.second);
      std::cout << "fact" << f << "|";
      std::cout << "var" << p.first << "=" << p.second << ", ";

      if (facts[f] == 0) {
        std::cout << std::endl << std::endl;
        std::cerr << "fact" << f << "|var" << p.first << "=" << p.second
                  << " is not satisfied for action " << o << " "
                  << sas->ActionName(o) << std::endl;
        exit(1);
      }
    }

    std::cout << std::endl << "eff: ";

    std::vector<std::pair<int, int> > effect;
    sas->CopyEffect(o, effect);

    for (auto p : effect) {
      int f = sas->Fact(p.first, p.second);
      std::cout << "fact" << f << "|";
      std::cout << "var" << p.first << "=" << p.second << ", ";
      facts[f] = 1;
    }

    std::cout << std::endl;

    if (sas->HasConditionalEffects(o)) {
      std::cout << "conditional eff: ";

      std::vector<std::vector<std::pair<int, int> > > effect_conditions;
      sas->CopyEffectConditions(o, effect_conditions);
      std::vector<std::pair<int, int> > conditional_effects;
      sas->CopyConditionalEffects(o, conditional_effects);

      for (int j = 0, m = effect_conditions.size(); j < m; ++j) {
        bool all = true;
        std::cout << "condition: ";

        for (auto &condition : effect_conditions[j]) {
          int f = sas->Fact(condition.first, condition.second);
          if (facts[f] == 0) all = false;
          std::cout << "fact" << f << "|";
          std::cout << "var" << condition.first << "=" << condition.second;
          std::cout << ",";
        }

        auto effect = conditional_effects[j];
        int f = sas->Fact(effect.first, effect.second);
        std::cout << " effect: ";
        std::cout << "fact" << f << "|";
        std::cout << "var" << effect.first << "=" << effect.second;

        if (all)
          facts[f] = 1;
        else
          std::cout << " not fired";

        std::cout << std::endl;
      }

      std::cout << std::endl;
    }

    std::cout << std::endl;
  }

  for (auto g : r_sas->goal()) {
    if (facts[g] == 0) {
      std::cerr << "goal fact " << g << " is not satisfied" << std::endl;
      exit(1);
    }
  }

  std::cout << "OK!" << std::endl;

  std::cout << "plan step: " << result.size() << std::endl;
  std::cout << "plan cost: " << cost << std::endl;
  std::cout << "number of preferred operators: " << preferred.size()
            << std::endl;
}
