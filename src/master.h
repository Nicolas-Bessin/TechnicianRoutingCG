// Include the gurobi api header
#pragma once
#include "solution.h"

MasterSolution relaxed_RMP(const Instance& instance, const std::vector<Route>& routes);

IntegerSolution integer_RMP(const Instance& instance, const std::vector<Route>& routes);