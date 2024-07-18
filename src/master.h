// Include the gurobi api header
#pragma once
#include "solution.h"

MasterSolution cg_solver(const Instance& instance, const std::vector<Route>& routes, double time_limit);

IntegerSolution solve_integer_problem(const Instance& instance, const std::vector<Route>& routes, double time_limit);